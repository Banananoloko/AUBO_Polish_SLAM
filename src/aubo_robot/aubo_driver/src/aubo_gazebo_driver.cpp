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
    std::lock_guard<std::mutex> lock(targets_mutex);
    for (int i = 0; i < ARM_DOF && i < (int)msg->position.size(); i++)
        joint_targets[i] = msg->position[i];
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

    ros::Rate rate(10);
    while (ros::ok())
    {
        if (shadow_mode)
        {
            // Detect time discontinuity (Gazebo pause/reset)
            ros::Time now = ros::Time::now();
            if (last_publish_time_.isValid())
            {
                double dt = (now - last_publish_time_).toSec();
                // Time jump: backward (reset) or forward >2s (long pause)
                if (dt < -0.1 || dt > 2.0)
                {
                    ROS_WARN("[aubo_gazebo_driver] Time discontinuity detected (dt=%.2fs), starting 2s blend to prevent PID divergence", dt);
                    blending_ = true;
                    blend_alpha_ = 0.0;

                    // Save current Gazebo positions as blend start (match by joint name)
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
                            {
                                // Fallback: use current target if Gazebo state unavailable
                                joint_start_[i] = joint_targets[i];
                            }
                        }
                    }
                    else
                    {
                        ROS_WARN("[aubo_gazebo_driver] Gazebo joint states not available, using targets as blend start");
                        std::lock_guard<std::mutex> lock(targets_mutex);
                        for (int i = 0; i < ARM_DOF; i++)
                            joint_start_[i] = joint_targets[i];
                    }
                }
            }
            last_publish_time_ = now;

            // Publish with blending if in protection mode
            std::lock_guard<std::mutex> lock(targets_mutex);
            if (blending_)
            {
                blend_alpha_ += 0.05; // 0.05 * 20 iterations = 1.0 over 2 seconds at 10Hz
                if (blend_alpha_ >= 1.0)
                {
                    blend_alpha_ = 1.0;
                    blending_ = false;
                    ROS_INFO("[aubo_gazebo_driver] Blend complete, resuming normal shadow mode");
                }

                // Smooth interpolation: start → target
                std_msgs::Float64 msg;
                for (int i = 0; i < ARM_DOF; i++)
                {
                    float blended = joint_start_[i] * (1.0 - blend_alpha_) + joint_targets[i] * blend_alpha_;
                    msg.data = blended;
                    pub_joints[i].publish(msg);
                }
            }
            else
            {
                // Normal operation
                publish_targets();
            }
        }
        rate.sleep();
    }

    spinner.stop();
    return 0;
}
