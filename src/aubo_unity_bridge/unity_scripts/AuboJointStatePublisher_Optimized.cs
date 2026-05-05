using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

/// <summary>
/// 优化版：精确 50Hz 发布关节状态，日志更简洁
/// 
/// 改进：
/// - 仅在启动、错误、状态变化时记录日志
/// - 移除每帧日志输出
/// - 添加性能监控（每 5 秒统计一次）
/// - 添加异常检测（关节角度突变）
/// </summary>
public class AuboJointStatePublisher : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/unity/joint_states";
    public float publishRateHz = 50f;

    [Header("Joints (顺序必须与 JointNames 严格一致)")]
    public ArticulationBody[] jointArticulations = new ArticulationBody[6];

    [Header("Monitoring")]
    public bool enablePerformanceLog = true;
    public float performanceLogInterval = 5f;

    public static readonly string[] JointNames = new[]
    {
        "shoulder_joint", "upperArm_joint", "foreArm_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint"
    };

    private ROSConnection ros;
    private JointStateMsg cachedMsg;
    private WaitForSeconds publishWait;
    
    // 性能监控
    private int publishCount = 0;
    private float lastLogTime = 0f;
    private double[] lastPositions = new double[6];
    private const double ANOMALY_THRESHOLD = 0.5; // 0.5 rad 突变阈值

    void Start()
    {
        if (!ValidateJoints()) return;

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);
        publishWait = new WaitForSeconds(1f / publishRateHz);

        cachedMsg = new JointStateMsg
        {
            header = new HeaderMsg(),
            name = JointNames,
            position = new double[6],
            velocity = new double[6],
            effort = new double[6]
        };

        Debug.Log($"[AuboJointStatePublisher] Started @ {publishRateHz} Hz on {topicName}");
        StartCoroutine(PublishLoop());
        
        if (enablePerformanceLog)
            StartCoroutine(PerformanceMonitor());
    }

    private bool ValidateJoints()
    {
        if (jointArticulations == null || jointArticulations.Length != 6)
        {
            Debug.LogError("[AuboJointStatePublisher] Requires exactly 6 ArticulationBody references");
            enabled = false;
            return false;
        }

        for (int i = 0; i < 6; i++)
        {
            if (jointArticulations[i] == null)
            {
                Debug.LogError($"[AuboJointStatePublisher] Joint {i} ({JointNames[i]}) is null");
                enabled = false;
                return false;
            }
        }
        return true;
    }

    private IEnumerator PublishLoop()
    {
        while (true)
        {
            PublishJointStates();
            yield return publishWait;
        }
    }

    private void PublishJointStates()
    {
        bool anomalyDetected = false;

        for (int i = 0; i < 6; i++)
        {
            var body = jointArticulations[i];
            double currentPos = body.jointPosition[0];
            
            // 异常检测（仅在有历史数据时）
            if (publishCount > 0 && System.Math.Abs(currentPos - lastPositions[i]) > ANOMALY_THRESHOLD)
            {
                anomalyDetected = true;
                Debug.LogWarning($"[AuboJointStatePublisher] Anomaly: {JointNames[i]} jumped " +
                    $"{System.Math.Abs(currentPos - lastPositions[i]):F3} rad");
            }

            cachedMsg.position[i] = currentPos;
            cachedMsg.velocity[i] = body.jointVelocity[0];
            cachedMsg.effort[i] = 0.0;
            lastPositions[i] = currentPos;
        }

        cachedMsg.header.stamp = new TimeMsg
        {
            sec = (uint)Mathf.FloorToInt(Time.time),
            nanosec = (uint)((Time.time - Mathf.FloorToInt(Time.time)) * 1e9f)
        };
        cachedMsg.header.frame_id = "base_link";

        ros.Publish(topicName, cachedMsg);
        publishCount++;
    }

    private IEnumerator PerformanceMonitor()
    {
        while (true)
        {
            yield return new WaitForSeconds(performanceLogInterval);
            
            float elapsed = Time.time - lastLogTime;
            float actualRate = publishCount / elapsed;
            
            Debug.Log($"[AuboJointStatePublisher] Published {publishCount} msgs in {elapsed:F1}s " +
                $"(actual rate: {actualRate:F1} Hz, target: {publishRateHz} Hz)");
            
            publishCount = 0;
            lastLogTime = Time.time;
        }
    }

    void OnDestroy()
    {
        StopAllCoroutines();
        Debug.Log("[AuboJointStatePublisher] Stopped");
    }
}
