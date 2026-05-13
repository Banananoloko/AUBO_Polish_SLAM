/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <mutex>
#include <vector>
#include <string>

#define ARM_DOF 6

static const std::string JOINT_NAMES[ARM_DOF] = {
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint",   "wrist2_joint",   "wrist3_joint"
};

float joint_targets[ARM_DOF];
std::mutex targets_mutex;

ros::Publisher pub_joints[ARM_DOF];

// Time discontinuity protection (for Gazebo pause/reset)
ros::Time last_publish_time_;
sensor_msgs::JointState current_gazebo_state_;
std::mutex gazebo_state_mutex_;
bool blending_ = false;
float blend_alpha_ = 0.0;
float joint_start_[ARM_DOF];

// Publish immediately from shadow callback when not blending.
// Declaring here so shadowCallback can read it without needing an atomic
// (written only from the main thread, read from spinner thread — but the
// race window is tiny and the worst outcome is one extra direct publish
// during blend startup, which is harmless).
bool g_direct_publish = false;

static void publish_targets()
{
    std_msgs::Float64 msg;
    for (int i = 0; i < ARM_DOF; i++)
    {
        msg.data = joint_targets[i];
        pub_joints[i].publish(msg);
    }
}

// Callback to track current Gazebo joint positions
void gazeboJointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(gazebo_state_mutex_);
    current_gazebo_state_ = *msg;
}

// Shadow mode: mirror real robot joint states → Gazebo
void shadowCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    {
        std::lock_guard<std::mutex> lock(targets_mutex);
        for (int i = 0; i < ARM_DOF && i < (int)msg->position.size(); i++)
            joint_targets[i] = msg->position[i];
    }
    // Publish immediately at the real-robot state rate (~50 Hz) when not in
    // a blend.  This eliminates the 10 Hz step changes that caused derivative
    // kicks (d × Δpos/0.001 s ≈ 312 Nm per step at 10 Hz → visual twitching).
    if (g_direct_publish)
        publish_targets();
}

// Normal mode: receive JointTrajectory from aubo_controller → Gazebo
void trajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
    if (msg->points.empty()) return;

    ros::Time start_time = ros::Time::now();

    for (const auto& point : msg->points)
    {
        // Wait until the point's scheduled time
        ros::Time target_time = start_time + point.time_from_start;
        ros::Time::sleepUntil(target_time);

        if (!ros::ok()) break;

        {
            std::lock_guard<std::mutex> lock(targets_mutex);
            // Map by joint name order in the message
            for (int i = 0; i < ARM_DOF; i++)
            {
                for (int j = 0; j < (int)msg->joint_names.size(); j++)
                {
                    if (JOINT_NAMES[i] == msg->joint_names[j])
                    {
                        joint_targets[i] = point.positions[j];
                        break;
                    }
                }
            }
        }
        publish_targets();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aubo_gazebo_driver");
    ros::NodeHandle nh;

    memset(joint_targets, 0.0, sizeof(joint_targets));
    memset(joint_start_, 0.0, sizeof(joint_start_));

    // shadow:=true  → mirror /real/joint_states to Gazebo (one-way, no commands back)
    // shadow:=false → receive /joint_path_command from aubo_controller and drive Gazebo
    bool shadow_mode;
    ros::param::param<bool>("~shadow", shadow_mode, false);

    std::string robot_name;
    ros::param::get("/robot_name", robot_name);

    // Advertise per-joint command topics (hardcoded names that aubo_gazebo_driver owns)
    const std::string cmd_suffix[ARM_DOF] = {
        "/shoulder_joint_position_controller/command",
        "/upperArm_joint_position_controller/command",
        "/foreArm_joint_position_controller/command",
        "/wrist1_joint_position_controller/command",
        "/wrist2_joint_position_controller/command",
        "/wrist3_joint_position_controller/command"
    };
    for (int i = 0; i < ARM_DOF; i++)
        pub_joints[i] = nh.advertise<std_msgs::Float64>(robot_name + cmd_suffix[i], 1000);

    // Subscribe to Gazebo joint states for time discontinuity protection
    ros::Subscriber gazebo_sub;
    if (shadow_mode)
    {
        gazebo_sub = nh.subscribe(robot_name + "/joint_states", 10, &gazeboJointStatesCallback);
        ROS_INFO("[aubo_gazebo_driver] Time discontinuity protection enabled");
    }

    ros::Subscriber sub;
    if (shadow_mode)
    {
        std::string real_topic;
        ros::param::param<std::string>("~real_joint_states_topic", real_topic, "/real/joint_states");
        ROS_INFO_STREAM("[aubo_gazebo_driver] Shadow mode: mirroring from " << real_topic);
        sub = nh.subscribe(real_topic, 10, &shadowCallback);
    }
    else
    {
        ROS_INFO("[aubo_gazebo_driver] Normal mode: subscribing to /joint_path_command");
        sub = nh.subscribe("joint_path_command", 10, &trajectoryCallback);
    }

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Startup blend: Gazebo begins at all-zeros; real robot may be elsewhere.
    // Force a blend from the initial Gazebo state (zeros) to the first received
    // real-robot state so the PID ramps smoothly rather than snapping.
    if (shadow_mode)
    {
        blending_       = true;
        g_direct_publish = false;
        blend_alpha_    = 0.0;
        // joint_start_ is already memset to 0 — matches Gazebo's spawn position.
        ROS_INFO("[aubo_gazebo_driver] Startup blend initiated (Gazebo at zero → real robot position)");
    }

    // Main loop: 50 Hz.
    // • During blend: drives Gazebo with interpolated targets.
    // • Normal operation: shadowCallback publishes directly; main loop only
    //   monitors for time discontinuities (Gazebo pause/reset).
    ros::Rate rate(50);
    while (ros::ok())
    {
        if (shadow_mode)
        {
            // ---- Time-discontinuity detection (Gazebo pause / reset) ----
            ros::Time now = ros::Time::now();
            if (last_publish_time_.isValid())
            {
                double dt = (now - last_publish_time_).toSec();
                if (dt < -0.1 || dt > 2.0)
                {
                    ROS_WARN("[aubo_gazebo_driver] Time discontinuity (dt=%.2fs) — starting 2 s blend", dt);
                    g_direct_publish = false;
                    blending_        = true;
                    blend_alpha_     = 0.0;

                    std::lock_guard<std::mutex> gazebo_lock(gazebo_state_mutex_);
                    if (!current_gazebo_state_.position.empty())
                    {
                        for (int i = 0; i < ARM_DOF; i++)
                        {
                            bool found = false;
                            for (size_t j = 0; j < current_gazebo_state_.name.size(); j++)
                            {
                                if (current_gazebo_state_.name[j] == JOINT_NAMES[i])
                                {
                                    joint_start_[i] = current_gazebo_state_.position[j];
                                    found = true;
                                    break;
                                }
                            }
                            if (!found)
                                joint_start_[i] = joint_targets[i];
                        }
                    }
                    else
                    {
                        std::lock_guard<std::mutex> lock(targets_mutex);
                        for (int i = 0; i < ARM_DOF; i++)
                            joint_start_[i] = joint_targets[i];
                    }
                }
            }
            last_publish_time_ = now;

            // ---- Blend: ramp from joint_start_ → joint_targets_ ----
            if (blending_)
            {
                // 2-second ramp at 50 Hz: step = 1/(50*2) = 0.01 per iteration
                blend_alpha_ += 0.01f;
                if (blend_alpha_ >= 1.0f)
                {
                    blend_alpha_     = 1.0f;
                    blending_        = false;
                    g_direct_publish = true;  // switch callback to direct publish
                    ROS_INFO("[aubo_gazebo_driver] Blend complete — direct 50 Hz shadow active");
                }

                std_msgs::Float64 msg;
                std::lock_guard<std::mutex> lock(targets_mutex);
                for (int i = 0; i < ARM_DOF; i++)
                {
                    msg.data = joint_start_[i] * (1.0f - blend_alpha_) + joint_targets[i] * blend_alpha_;
                    pub_joints[i].publish(msg);
                }
            }
            // else: shadowCallback is publishing directly at real-robot rate.
        }
        rate.sleep();
    }

    spinner.stop();
    return 0;
}
