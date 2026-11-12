#nullable enable

using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Abb.Egm;
using Google.Protobuf;

namespace WPFProject
{
    /// <summary>
    /// EGM UDP 通信客户端：
    /// - 绑定本地端口接收机器人反馈，并维护“会话健康度”（反馈超时 → DISCONNECTED）
    /// - 以后台固定周期向机器人发送最新目标位姿
    /// - 发送看门狗：目标超过 <see cref="TargetStaleMs"/> 未刷新时，切换为“保持当前位置”
    ///
    /// 说明：EGM pose guidance 要求外部设备周期性发送修正消息（典型 4~10 ms 一帧）。
    /// 本类把“更新目标”和“周期发送”解耦：UI 只更新目标，发送线程持续工作。
    /// 不涉及任何 UI 逻辑。
    /// </summary>
    public sealed class EgmClient : IDisposable
    {
        /// <summary>超过该时长没有收到新反馈，则判定为 DISCONNECTED。</summary>
        public const int FeedbackTimeoutMs = 200;

        /// <summary>目标超过该时长未被 UI 刷新，则切换为保持当前位置。</summary>
        public const int TargetStaleMs = 500;

        // 发送周期（毫秒）。EGM 默认采样周期为 4 ms；受系统计时器精度影响，
        // 实际速率通常为 ~60~100 Hz，远高于 EGM 超时阈值。
        private const int StreamPeriodMs = 4;

        // 会话健康检查周期（毫秒）。用于在没有新帧时也能及时触发超时状态。
        private const int HealthPeriodMs = 20;

        // Windows 上禁止 UDP 端口不可达时产生的 ConnectionReset，避免接收循环被无谓打断。
        private const int SioUdpConnReset = -1744830452;

        private readonly UdpClient _udp;
        private readonly CancellationTokenSource _cts = new();
        private readonly object _targetLock = new();
        private readonly object _feedbackLock = new();
        private readonly object _sessionLock = new();

        private Task? _receiveTask;
        private Task? _streamTask;
        private Task? _healthTask;

        private IPEndPoint? _robotEndPoint;
        private uint _sequenceNumber;

        // 最新目标位姿（UI 线程写入，发送线程读取）
        private bool _hasTarget;
        private bool _targetExpired;
        private double _tx, _ty, _tz, _trx, _try_, _trz;
        private long _lastTargetUpdateTicks;

        // 反馈健康度
        private long _lastFeedbackTicks;
        private long _frameCount;
        private int _lastMciRunning; // 0/1，最后一条有效反馈是否报告 MCI_RUNNING
        private RobotState _lastFeedback;
        private bool _hasFeedback;

        private EgmState _sessionState = EgmState.Disconnected;
        private int _needsTargetResync = 1;
        private int _started;
        private int _disposed;

        /// <summary>收到一帧有效机器人状态时触发（后台线程）。</summary>
        public event Action<RobotState>? StateReceived;

        /// <summary>会话状态发生变化时触发（后台线程）。</summary>
        public event Action<EgmState>? SessionStateChanged;

        /// <summary>收到无法解析的消息时触发（后台线程）。</summary>
        public event Action<Exception>? ParseError;

        /// <summary>发送发生错误时触发（可能来自发送线程）。</summary>
        public event Action<Exception>? SendError;

        /// <summary>目标超过 <see cref="TargetStaleMs"/> 未刷新时触发一次（后台线程）。</summary>
        public event Action? TargetExpired;

        /// <summary>会话恢复 / 首次连接时，客户端把目标重置为最近反馈位姿后触发。</summary>
        public event Action<RobotState>? TargetResynced;

        public EgmClient(int port)
        {
            _udp = new UdpClient(new IPEndPoint(IPAddress.Any, port));

            // 仅 Windows 支持该 IOControl；失败时忽略，不影响基本通信。
            try
            {
                _udp.Client.IOControl(SioUdpConnReset, new byte[] { 0, 0, 0, 0 }, null);
            }
            catch
            {
                // ignore
            }
        }

        /// <summary>
        /// 是否处于健康会话中（最近 <see cref="FeedbackTimeoutMs"/> 内收到过反馈）。
        /// 注意：这不再表示“曾经收到过一帧”，而是反映当前会话健康度。
        /// </summary>
        public bool IsRobotConnected => SessionState != EgmState.Disconnected;

        /// <summary>当前会话状态（DISCONNECTED / CONNECTED / RUNNING）。</summary>
        public EgmState SessionState
        {
            get
            {
                lock (_sessionLock)
                {
                    return _sessionState;
                }
            }
        }

        /// <summary>机器人远端地址；尚未收到反馈时为 null。</summary>
        public IPEndPoint? RemoteEndPoint => _robotEndPoint;

        /// <summary>累计收到的 UDP 帧数（包括解析失败的帧）。</summary>
        public long FrameCount => Interlocked.Read(ref _frameCount);

        /// <summary>距离最近一帧反馈的毫秒数；从未收到反馈时为 -1。</summary>
        public int LastFeedbackAgeMs
        {
            get
            {
                long last = Interlocked.Read(ref _lastFeedbackTicks);
                if (last == 0)
                {
                    return -1;
                }

                double elapsed = ElapsedMilliseconds(last);
                return elapsed > int.MaxValue ? int.MaxValue : (int)elapsed;
            }
        }

        /// <summary>是否已有可发送的目标。</summary>
        public bool HasTarget
        {
            get
            {
                lock (_targetLock)
                {
                    return _hasTarget;
                }
            }
        }

        /// <summary>目标是否已被发送看门狗标记为过期（此时客户端发送保持位姿）。</summary>
        public bool IsTargetExpired
        {
            get
            {
                lock (_targetLock)
                {
                    return _targetExpired;
                }
            }
        }

        /// <summary>启动后台接收、发送和健康检查循环。</summary>
        public void Start()
        {
            if (Interlocked.CompareExchange(ref _started, 1, 0) != 0)
            {
                return;
            }

            _receiveTask = Task.Run(ReceiveLoopAsync);
            _streamTask = Task.Run(StreamLoopAsync);
            _healthTask = Task.Run(HealthLoopAsync);
        }

        /// <summary>
        /// 更新目标位姿（mm / 度）。实际发送由后台流循环以固定周期完成，
        /// 因此重复调用是安全的（连点只会覆盖目标）。
        /// 显式设置目标会同时清除“目标过期”锁存并刷新目标时间戳。
        /// </summary>
        public void SetTarget(
            double x, double y, double z,
            double rx, double ry, double rz)
        {
            if (!double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(z)
                || !double.IsFinite(rx) || !double.IsFinite(ry) || !double.IsFinite(rz))
            {
                throw new ArgumentOutOfRangeException(nameof(x), "目标位姿必须是有限数值。");
            }

            SetTargetCore(x, y, z, rx, ry, rz);
        }

        /// <summary>
        /// UI 心跳：在目标未过期时刷新“最后目标更新时间”。
        /// 若 UI 线程卡死超过 <see cref="TargetStaleMs"/>，发送线程不再续租，
        /// 自动切换为保持当前位置。
        /// </summary>
        public void RefreshTargetLease()
        {
            lock (_targetLock)
            {
                if (!_hasTarget || _targetExpired)
                {
                    return;
                }

                _lastTargetUpdateTicks = Stopwatch.GetTimestamp();
            }
        }

        /// <summary>获取最近一帧有效反馈的副本。</summary>
        public bool TryGetLatestFeedback(out RobotState state)
        {
            lock (_feedbackLock)
            {
                state = _lastFeedback;
                return _hasFeedback;
            }
        }

        private void SetTargetCore(
            double x, double y, double z,
            double rx, double ry, double rz)
        {
            lock (_targetLock)
            {
                _tx = x; _ty = y; _tz = z;
                _trx = rx; _try_ = ry; _trz = rz;
                _hasTarget = true;
                _targetExpired = false;
                _lastTargetUpdateTicks = Stopwatch.GetTimestamp();
            }
        }

        private async Task ReceiveLoopAsync()
        {
            while (!_cts.IsCancellationRequested)
            {
                try
                {
                    UdpReceiveResult result = await _udp
                        .ReceiveAsync(_cts.Token)
                        .ConfigureAwait(false);

                    // 每次收到消息都刷新机器人地址，兼容机器人 IP / 端口变化。
                    _robotEndPoint = new IPEndPoint(
                        result.RemoteEndPoint.Address,
                        result.RemoteEndPoint.Port);

                    // 只要收到来自机器人的 UDP 包，就说明链路仍活着；
                    // 先刷新反馈时间，再判断内容是否有效。
                    Interlocked.Exchange(ref _lastFeedbackTicks, Stopwatch.GetTimestamp());
                    Interlocked.Increment(ref _frameCount);

                    EgmRobot message;
                    try
                    {
                        message = EgmRobot.Parser.ParseFrom(result.Buffer);
                    }
                    catch (InvalidProtocolBufferException ex)
                    {
                        // 内容坏掉时不再宣称 RUNNING，避免在未知状态下允许移动。
                        Volatile.Write(ref _lastMciRunning, 0);
                        ParseError?.Invoke(ex);
                        UpdateSessionState();
                        continue;
                    }

                    if (!EgmProtocol.TryParseRobotState(message, out RobotState state))
                    {
                        Volatile.Write(ref _lastMciRunning, 0);
                        ParseError?.Invoke(new InvalidOperationException(
                            "反馈帧缺少 header / feedback.cartesian / 欧拉角字段。"));
                        UpdateSessionState();
                        continue;
                    }

                    Volatile.Write(
                        ref _lastMciRunning,
                        state.EgmState == EgmState.Running ? 1 : 0);

                    lock (_feedbackLock)
                    {
                        _lastFeedback = state;
                        _hasFeedback = true;
                    }

                    // 首次连接或断开恢复后的第一帧：把目标重置为反馈位姿。
                    // 这个动作绑定在“有效反馈帧”上，而不是状态切换事件上，
                    // 避免健康检查线程抢先完成 Disconnected -> Connected 切换，
                    // 导致目标未被重置而在恢复瞬间跳跃。
                    if (Interlocked.Exchange(ref _needsTargetResync, 0) == 1)
                    {
                        SetTargetCore(
                            state.X, state.Y, state.Z,
                            state.Rx, state.Ry, state.Rz);


                        try
                        {
                            TargetResynced?.Invoke(state);
                        }
                        catch (Exception ex)
                        {
                            Debug.WriteLine($"[EgmCiient] TargetResynced handler failed: {ex.Message}");
                        }
                    }

                    UpdateSessionState();

                    try
                    {
                        StateReceived?.Invoke(state);
                    }
                    catch (Exception ex)
                    {
                        // UI 回调异常不应终止接收线程。
                        ParseError?.Invoke(ex);
                    }
                }
                catch (OperationCanceledException)
                {
                    break; // 正常关闭
                }
                catch (ObjectDisposedException)
                {
                    break; // socket 已被 Dispose
                }
                catch (SocketException ex)
                {
                    // 瞬时错误：稍等后重试；ConnectionReset 已在构造时尽量关闭。
                    Debug.WriteLine($"[EgmClient] SocketException: {ex.SocketErrorCode}");
                    if (ex.SocketErrorCode != SocketError.ConnectionReset)
                    {
                        try
                        {
                            await Task.Delay(100, _cts.Token).ConfigureAwait(false);
                        }
                        catch (OperationCanceledException)
                        {
                            break;
                        }
                    }
                }
                catch (Exception ex)
                {
                    // 非致命异常记录后继续
                    ParseError?.Invoke(ex);
                }
            }
        }

        private async Task StreamLoopAsync()
        {
            while (!_cts.IsCancellationRequested)
            {
                try
                {
                    EgmState session = SessionState;
                    IPEndPoint? endpoint = _robotEndPoint;

                    // 反馈超时后停止发送，避免继续向一个已失联的会话推送目标。
                    if (session == EgmState.Disconnected || endpoint == null)
                    {
                        await Task.Delay(StreamPeriodMs, _cts.Token).ConfigureAwait(false);
                        continue;
                    }

                    bool hasTarget;
                    bool expired;
                    bool fireExpired = false;
                    double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;

                    lock (_targetLock)
                    {
                        hasTarget = _hasTarget;
                        expired = hasTarget
                            && ElapsedMilliseconds(_lastTargetUpdateTicks) > TargetStaleMs;

                        if (expired)
                        {
                            if (!_targetExpired)
                            {
                                _targetExpired = true;
                                fireExpired = true;
                            }
                        }
                        else if (hasTarget)
                        {
                            _targetExpired = false;
                            x = _tx; y = _ty; z = _tz;
                            rx = _trx; ry = _try_; rz = _trz;
                        }
                    }

                    if (hasTarget)
                    {
                        if (expired)
                        {
                            // 目标过期：保持最近反馈位置，而不是继续执行旧目标。
                            if (TryGetLatestFeedback(out RobotState hold))
                            {
                                SendPose(endpoint, hold.X, hold.Y, hold.Z, hold.Rx, hold.Ry, hold.Rz);
                            }
                        }
                        else
                        {
                            SendPose(endpoint, x, y, z, rx, ry, rz);
                        }
                    }

                    if (fireExpired)
                    {
                        try
                        {
                            TargetExpired?.Invoke();
                        }
                        catch (Exception ex)
                        {
                            Debug.WriteLine($"[EgmClient] TargetExpired handler failed: {ex.Message}");
                        }
                    }

                    await Task.Delay(StreamPeriodMs, _cts.Token).ConfigureAwait(false);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    try
                    {
                        SendError?.Invoke(ex);
                    }
                    catch (Exception handlerEx)
                    {
                        Debug.WriteLine($"[EgmClient] SendError handler failed: {handlerEx.Message}");
                    }

                    try
                    {
                        await Task.Delay(100, _cts.Token).ConfigureAwait(false);
                    }
                    catch (OperationCanceledException)
                    {
                        break;
                    }
                }
            }
        }

        private async Task HealthLoopAsync()
        {
            while (!_cts.IsCancellationRequested)
            {
                try
                {
                    UpdateSessionState();
                    await Task.Delay(HealthPeriodMs, _cts.Token).ConfigureAwait(false);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    Debug.WriteLine($"[EgmClient] Health loop: {ex.Message}");
                    try
                    {
                        await Task.Delay(100, _cts.Token).ConfigureAwait(false);
                    }
                    catch (OperationCanceledException)
                    {
                        break;
                    }
                }
            }
        }

        private void SendPose(
            IPEndPoint endpoint,
            double x, double y, double z,
            double rx, double ry, double rz)
        {
            _sequenceNumber++;

            EgmSensor message = EgmProtocol.BuildSensorMessage(
                _sequenceNumber, x, y, z, rx, ry, rz);

            byte[] data = message.ToByteArray();
            _udp.Send(data, data.Length, endpoint);
        }

        // 计算当前会话状态（DISCONNECTED / CONNECTED / RUNNING）。
        private EgmState ComputeSessionState()
        {
            if (_robotEndPoint == null)
            {
                return EgmState.Disconnected;
            }

            long lastFeedback = Interlocked.Read(ref _lastFeedbackTicks);
            if (lastFeedback == 0
                || ElapsedMilliseconds(lastFeedback) > FeedbackTimeoutMs)
            {
                return EgmState.Disconnected;
            }

            return Volatile.Read(ref _lastMciRunning) != 0
                ? EgmState.Running
                : EgmState.Connected;
        }

        private void UpdateSessionState()
        {
            EgmState oldState;
            EgmState newState;

            lock (_sessionLock)
            {
                // 在锁内计算，避免健康检查线程拿着“旧快照”覆盖接收线程刚写入的新状态。
                newState = ComputeSessionState();
                oldState = _sessionState;
                if (oldState == newState)
                {
                    return;
                }

                _sessionState = newState;
            }

            if (newState == EgmState.Disconnected)
            {
                // 断开后恢复时，必须用新的有效反馈重新初始化目标。
                Interlocked.Exchange(ref _needsTargetResync, 1);
            }
            else if (newState == EgmState.Running && oldState != EgmState.Running)
            {
                // 从 CONNECTED 重新进入 RUNNING 时也可能已经发生过外部移动，
                // 在下一帧把目标重置为反馈，避免执行进入 RUNNING 前的旧目标。
                Interlocked.Exchange(ref _needsTargetResync, 1);
            }

            try
            {
                SessionStateChanged?.Invoke(newState);
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[EgmClient] SessionStateChanged handler failed: {ex.Message}");
            }
        }

        private static double ElapsedMilliseconds(long sinceTimestamp)
        {
            return (Stopwatch.GetTimestamp() - sinceTimestamp) * 1000.0 / Stopwatch.Frequency;
        }

        public void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) != 0)
            {
                return;
            }

            try { _cts.Cancel(); } catch { /* ignore */ }
            try { _udp.Dispose(); } catch { /* ignore */ }

            Task?[] tasks = { _receiveTask, _streamTask, _healthTask };
            foreach (Task? task in tasks)
            {
                if (task == null)
                {
                    continue;
                }

                try { task.Wait(TimeSpan.FromSeconds(1)); } catch { /* ignore */ }
            }

            _cts.Dispose();
        }
    }
}
