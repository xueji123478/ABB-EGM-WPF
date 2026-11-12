#nullable enable

using System;
using Abb.Egm;

namespace WPFProject
{
    /// <summary>
    /// EGM 会话状态（由 EgmClient 结合“反馈是否新鲜”和反馈中的 MCI 状态维护）。
    /// </summary>
    public enum EgmState
    {
        /// <summary>尚未收到反馈，或反馈超过超时阈值。</summary>
        Disconnected,

        /// <summary>反馈正常，但控制器尚未进入 RUNNING（例如等待 EGMRunPose）。</summary>
        Connected,
        
        /// <summary>反馈正常且 MCI 状态为 RUNNING，可以下发运动目标。</summary>
        Running
    }

    /// <summary>
    /// EGM 消息构造工具。
    /// 集中处理 Header / Planned / FeedBack 的组装与解析，避免散落在 UI 层。
    /// </summary>
    internal static class EgmProtocol
    {
        /// <summary>
        /// 构造一帧 EgmSensor（发送给机器人）。
        /// </summary>
        /// <param name="sequenceNumber">消息序号（调用方自增）</param>
        /// <param name="x">X 位置，单位 mm</param>
        /// <param name="y">Y 位置，单位 mm</param>
        /// <param name="z">Z 位置，单位 mm</param>
        /// <param name="rx">绕 X 的欧拉角，单位 度</param>
        /// <param name="ry">绕 Y 的欧拉角，单位 度</param>
        /// <param name="rz">绕 Z 的欧拉角，单位 度</param>
        public static EgmSensor BuildSensorMessage(
            uint sequenceNumber,
            double x, double y, double z,
            double rx, double ry, double rz)
        {
            EgmHeader header = new EgmHeader
            {
                Seqno = sequenceNumber,
                // EGM 的 tm 为毫秒时间戳；uint 回绕（约 49 天）对控制器没有影响。
                Tm = unchecked((uint)Environment.TickCount64),
                Mtype = EgmHeader.Types.MessageType.MsgtypeCorrection
            };

            EgmCartesian pos = new EgmCartesian { X = x, Y = y, Z = z };
            EgmEuler euler = new EgmEuler { X = rx, Y = ry, Z = rz };

            EgmPose pose = new EgmPose { Pos = pos, Euler = euler };
            EgmPlanned planned = new EgmPlanned { Cartesian = pose };

            return new EgmSensor
            {
                Header = header,
                Planned = planned
            };
        }

        /// <summary>
        /// 从机器人反馈消息中解析位置、欧拉角、6 轴关节角与 MCI 状态。
        /// 关键字段缺失时返回 false，由调用方记录解析错误。
        /// </summary>
        public static bool TryParseRobotState(
            EgmRobot? message,
            out RobotState state)
        {
            state = default;

            if (message?.Header == null
                || !message.Header.HasSeqno
                || !message.Header.HasTm)
            {
                return false;
            }

            EgmFeedBack? feedback = message.FeedBack;
            EgmPose? cartesian = feedback?.Cartesian;
            if (cartesian?.Pos == null || cartesian.Euler == null)
            {
                return false;
            }

            // NaN / Infinity 不应进入目标流或 UI。
            if (!double.IsFinite(cartesian.Pos.X)
                || !double.IsFinite(cartesian.Pos.Y)
                || !double.IsFinite(cartesian.Pos.Z)
                || !double.IsFinite(cartesian.Euler.X)
                || !double.IsFinite(cartesian.Euler.Y)
                || !double.IsFinite(cartesian.Euler.Z))
            {
                return false;
            }

            // EgmFeedBack.joints 是 repeated double。部分配置可能不发送关节角，
            // 此时保留空数组，UI 显示为“—”，不影响笛卡尔位姿控制。
            double[] joints = Array.Empty<double>();
            var repeatedJoints = feedback!.Joints?.Joints;
            if (repeatedJoints != null && repeatedJoints.Count > 0)
            {
                joints = new double[repeatedJoints.Count];
                repeatedJoints.CopyTo(joints, 0);
            }

            var mciCode = message.MciState?.State
                          ?? EgmMCIState.Types.MCIStateType.MciUndefined;

            string mciText = mciCode switch
            {
                EgmMCIState.Types.MCIStateType.MciUndefined => "MCI_UNDEFINED",
                EgmMCIState.Types.MCIStateType.MciError => "MCI_ERROR",
                EgmMCIState.Types.MCIStateType.MciStopped => "MCI_STOPPED",
                EgmMCIState.Types.MCIStateType.MciRunning => "MCI_RUNNING",
                _ => "MCI_UNKNOWN"
            };

            // 帧内的 MCI 状态只是“连接/运行”判断的一部分；
            // EgmClient 还会叠加 200 ms 反馈超时，才会给出最终 EgmState。
            EgmState frameState = mciCode == EgmMCIState.Types.MCIStateType.MciRunning
                ? EgmState.Running
                : EgmState.Connected;

            state = new RobotState
            {
                X = cartesian.Pos.X,
                Y = cartesian.Pos.Y,
                Z = cartesian.Pos.Z,
                Rx = cartesian.Euler.X,
                Ry = cartesian.Euler.Y,
                Rz = cartesian.Euler.Z,
                Joints = joints,
                MciState = mciText,
                EgmState = frameState,
                Converged = message.HasMciConvergenceMet
                    ? (bool?)message.MciConvergenceMet
                    : null,
                Seqno = message.Header.Seqno
            };
            return true;
        }
    }

    /// <summary>机器人反馈状态的不可变快照。</summary>
    public readonly struct RobotState
    {
        public double X { get; init; }
        public double Y { get; init; }
        public double Z { get; init; }
        public double Rx { get; init; }
        public double Ry { get; init; }
        public double Rz { get; init; }

        /// <summary>6 轴关节角（度）；反馈未携带时为长度为 0 的数组。</summary>
        public double[] Joints { get; init; }

        /// <summary>反馈帧中的 MCI 状态文本，例如 MCI_RUNNING。</summary>
        public string MciState { get; init; }

        /// <summary>仅根据本帧 MCI 得出的状态；会话级状态以 EgmClient.SessionState 为准。</summary>
        public EgmState EgmState { get; init; }

        /// <summary>MCI 是否报告已收敛；字段未提供时为 null。</summary>
        public bool? Converged { get; init; }

        /// <summary>反馈帧序号。</summary>
        public uint Seqno { get; init; }
    }
}
