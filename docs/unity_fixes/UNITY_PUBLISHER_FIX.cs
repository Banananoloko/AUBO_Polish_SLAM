using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

/// <summary>
/// 精确 50Hz 发布 6 关节状态到 ROS 话题 /unity/joint_states (sensor_msgs/JointState)
/// 对标 Gazebo 的 joint_state_controller。
///
/// 修复说明：
/// - 使用 Coroutine 而非 Update() 确保精确的 50 Hz 发布频率
/// - 避免受游戏帧率影响
/// - 使用 WaitForSeconds 精确控制时间间隔
///
/// 使用：将此脚本挂到任意 GameObject（比如机器人根 GameObject 或 ROSConnection 物体），
///      在 Inspector 中：
///        - Joint Articulations 数组按顺序拖入 6 个 ArticulationBody（与 JointNames 顺序严格对应）
///        - Topic Name 默认 "/unity/joint_states"
///        - Publish Rate Hz 默认 50
/// </summary>
public class AuboJointStatePublisher : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/unity/joint_states";
    public float publishRateHz = 50f;

    [Header("Joints (顺序必须与 JointNames 严格一致)")]
    public ArticulationBody[] jointArticulations = new ArticulationBody[6];

    public static readonly string[] JointNames = new[]
    {
        "shoulder_joint",
        "upperArm_joint",
        "foreArm_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint"
    };

    private ROSConnection ros;
    private JointStateMsg cachedMsg;
    private WaitForSeconds publishWait;

    void Start()
    {
        if (jointArticulations == null || jointArticulations.Length != 6)
        {
            Debug.LogError("[AuboJointStatePublisher] Need exactly 6 ArticulationBody references.");
            enabled = false;
            return;
        }

        for (int i = 0; i < 6; i++)
        {
            if (jointArticulations[i] == null)
            {
                Debug.LogError($"[AuboJointStatePublisher] jointArticulations[{i}] ({JointNames[i]}) is null.");
                enabled = false;
                return;
            }
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);

        // 使用 WaitForSeconds 精确控制发布间隔
        publishWait = new WaitForSeconds(1f / publishRateHz);

        cachedMsg = new JointStateMsg
        {
            header = new HeaderMsg(),
            name = JointNames,
            position = new double[6],
            velocity = new double[6],
            effort = new double[6]
        };

        Debug.Log($"[AuboJointStatePublisher] Publishing {topicName} @ {publishRateHz} Hz (using Coroutine)");

        // 启动发布协程
        StartCoroutine(PublishLoop());
    }

    /// <summary>
    /// 精确 50 Hz 发布循环（使用 Coroutine）
    /// </summary>
    private IEnumerator PublishLoop()
    {
        while (true)
        {
            PublishJointStates();
            yield return publishWait;  // 精确等待 1/50 = 0.02 秒
        }
    }

    /// <summary>
    /// 发布关节状态
    /// </summary>
    private void PublishJointStates()
    {
        for (int i = 0; i < 6; i++)
        {
            var body = jointArticulations[i];
            // ArticulationBody 的关节角度（弧度）
            cachedMsg.position[i] = body.jointPosition[0];
            cachedMsg.velocity[i] = body.jointVelocity[0];
            cachedMsg.effort[i] = 0.0;  // 力矩可选；ArticulationBody 不直接提供，留 0
        }

        // 时间戳：使用 Unity 当前时间（ROS 端会 restamp）
        cachedMsg.header.stamp = new TimeMsg
        {
            sec = (uint)Mathf.FloorToInt(Time.time),
            nanosec = (uint)((Time.time - Mathf.FloorToInt(Time.time)) * 1e9f)
        };
        cachedMsg.header.frame_id = "base_link";

        ros.Publish(topicName, cachedMsg);
    }

    void OnDestroy()
    {
        // 停止协程
        StopAllCoroutines();
    }
}
