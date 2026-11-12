#nullable enable

using System;
using System.ComponentModel;
using System.Globalization;
using System.Windows;
using System.Windows.Media;
using System.Windows.Threading;

namespace WPFProject
{
    /// <summary>
    /// 主窗口：
    /// - 通过 EgmClient 与机器人通信
    /// - 维护“本地目标位姿”，点击按钮 / 应用输入后更新目标
    /// - 定时刷新反馈位姿、关节角、会话健康度与目标偏差
    /// </summary>
    public partial class MainWindow : Window
    {
        // ---------- 配置 ----------
        private const int Port = 6510;
        private const double TranslationIncrement = 10.0; // mm
        private const double RotationIncrement = 2.0;     // 度

        // 单次目标与当前反馈的最大允许偏差（安全限幅）
        private const double MaxTranslationDeviationMm = 50.0;
        private const double MaxRotationDeviationDeg = 10.0;
        
        // ---------- 颜色 ----------
        private static readonly Brush TextBrush = Frozen(0x0F, 0x17, 0x2A);
        private static readonly Brush MutedBrush = Frozen(0x64, 0x74, 0x8B);
        private static readonly Brush OkBrush = Frozen(0x15, 0x80, 0x3D);
        private static readonly Brush WarnBrush = Frozen(0xB4, 0x53, 0x09);
        private static readonly Brush ErrorBrush = Frozen(0xB9, 0x1C, 0x1C);
        private static readonly Brush InfoBrush = Frozen(0x33, 0x41, 0x55);

        private static readonly Brush StateDisconnectedBrush = Frozen(0xDC, 0x26, 0x26);
        private static readonly Brush StateConnectedBrush = Frozen(0xD9, 0x77, 0x06);
        private static readonly Brush StateRunningBrush = Frozen(0x16, 0xA3, 0x4A);

        private static readonly Brush StatusInfoBrush = Brushes.White;
        private static readonly Brush StatusOkBrush = Frozen(0x86, 0xEF, 0xAC);
        private static readonly Brush StatusWarnBrush = Frozen(0xFD, 0xE6, 0x8A);
        private static readonly Brush StatusErrorBrush = Frozen(0xFE, 0xCA, 0xCA);

        // ---------- 通信 ----------
        private EgmClient? _egm;
        private readonly DispatcherTimer _uiTimer;
        private readonly DispatcherTimer _leaseTimer;

        // ---------- 本地目标位置（发送依据） ----------
        private readonly object _stateLock = new();
        private double _tx, _ty, _tz;
        private double _trx, _try_, _trz;

        // ---------- 当前机器人反馈（显示依据） ----------
        private RobotState _lastState;
        private bool _hasLastState;
        private bool _targetInputsInitialized;

        // ---------- UI / 日志状态 ----------
        private readonly object _errorLock = new();
        private EgmState _uiSessionState = EgmState.Disconnected;
        private bool _hasEverConnected;
        private string _lastErrorSignature = string.Empty;
        private DateTime _lastErrorUtc = DateTime.MinValue;
        private bool _closing;

        public MainWindow()
        {
            InitializeComponent();

            
            _uiTimer = new DispatcherTimer(DispatcherPriority.Background)
            {
                Interval = TimeSpan.FromMilliseconds(50)
            };
            _uiTimer.Tick += OnUiTimerTick;

            _leaseTimer = new DispatcherTimer(DispatcherPriority.Background)
            {
                Interval = TimeSpan.FromMilliseconds(100)
            };
            _leaseTimer.Tick += OnLeaseTimerTick;

            try
            {
                _egm = new EgmClient(Port);
                // 事件处理器在后台线程触发，需 Post 到 UI 线程更新界面。
                _egm.StateReceived += OnRobotStateReceived;
                // SessionStateChanged 事件在后台线程触发，需 Post 到 UI 线程更新界面。
                _egm.SessionStateChanged += OnSessionStateChanged;
                // TargetExpired / TargetResynced 事件在后台线程触发，需 Post 到 UI 线程更新界面。
                _egm.TargetExpired += OnTargetExpired;
                // TargetResynced 事件在后台线程触发，需 Post 到 UI 线程更新界面。
                _egm.TargetResynced += OnTargetResynced;
                // ParseError / SendError 事件在后台线程触发，需 Post 到 UI 线程更新界面。
                _egm.ParseError += OnParseError;
                // SendError 事件在后台线程触发，需 Post 到 UI 线程更新界面。
                _egm.SendError += OnSendError;
                _egm.Start();

                UpdateSessionVisuals(EgmState.Disconnected);
                Log($"EGM 客户端已启动，UDP 监听端口 {Port}，等待机器人反馈…", InfoBrush);
                SetStatus("等待 EGM 反馈…", StatusInfoBrush);
            }
            catch (Exception ex)
            {
                UpdateSessionVisuals(EgmState.Disconnected);
                Log("EGM 客户端启动失败：" + ex.Message, ErrorBrush);
                SetStatus("启动失败：" + ex.Message, StatusErrorBrush);
                MessageBox.Show(
                    "EGM 客户端启动失败：\n\n" + ex.Message,
                    "启动失败",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
            }

            _uiTimer.Start();
            _leaseTimer.Start();
        }

        // -----------------------------------------------------------------
        // 机器人反馈：保存快照，并在首次收到反馈时初始化目标
        // -----------------------------------------------------------------
        private void OnRobotStateReceived(RobotState state)
        {
            EgmClient? egm = _egm;
            if (egm is null)
            {
                return;
            }

            lock (_stateLock)
            {
                if (!_hasLastState)
                {
                    _hasLastState = true;
                    _tx = state.X; _ty = state.Y; _tz = state.Z;
                    _trx = state.Rx; _try_ = state.Ry; _trz = state.Rz;

                    // 首次反馈时用机器人当前位置初始化目标，并开始周期性发送，
                    // 既保持 EGM 会话，又避免从 0 位置突然跳变。
                    egm.SetTarget(_tx, _ty, _tz, _trx, _try_, _trz);
                }

                // 注意：不把目标“吸附”到反馈值。目标是绝对指令，
                // 反馈与目标的差异通过 Δ位置 / Δ姿态 显示，避免逐步丢失指令位置。

                _lastState = state;
            }
        }

        // -----------------------------------------------------------------
        // 会话状态变化 / 看门狗事件（均来自后台线程）
        // -----------------------------------------------------------------
        private void OnSessionStateChanged(EgmState state)
        {
            PostToUi(() =>
            {
                EgmState previous = _uiSessionState;
                _uiSessionState = state;
                UpdateSessionVisuals(state);

                if (previous == state)
                {
                    return;
                }

                switch (state)
                {
                    case EgmState.Running:
                        Log("EGM 状态：RUNNING，移动命令已启用。", OkBrush);
                        SetStatus("EGM RUNNING：可点动或应用目标位姿。", StatusOkBrush);
                        break;

                    case EgmState.Connected:
                        Log("EGM 状态：CONNECTED，等待控制器进入 RUNNING。", WarnBrush);
                        SetStatus("EGM CONNECTED：等待控制器进入 RUNNING，移动暂不可用。", StatusWarnBrush);
                        break;

                    default:
                        Log($"EGM 状态：DISCONNECTED（{EgmClient.FeedbackTimeoutMs} ms 未收到反馈），发送已暂停。", ErrorBrush);
                        SetStatus("EGM DISCONNECTED：反馈超时，发送已暂停，移动已禁用。", StatusErrorBrush);
                        break;
                }

                if (previous == EgmState.Disconnected && state != EgmState.Disconnected)
                {
                    if (_hasEverConnected)
                    {
                        Log("EGM 会话恢复：目标已重置为最近反馈位姿，避免位置跳跃。", InfoBrush);
                    }

                    _hasEverConnected = true;
                }
            });
        }

        private void OnTargetExpired()
        {
            PostToUi(() =>
            {
                Log($"发送看门狗：目标超过 {EgmClient.TargetStaleMs} ms 未刷新，已切换为保持当前位置。", WarnBrush);

                EgmClient? egm = _egm;
                if (egm is null || !egm.TryGetLatestFeedback(out RobotState feedback))
                {
                    return;
                }

                lock (_stateLock)
                {
                    _tx = feedback.X; _ty = feedback.Y; _tz = feedback.Z;
                    _trx = feedback.Rx; _try_ = feedback.Ry; _trz = feedback.Rz;
                    egm.SetTarget(_tx, _ty, _tz, _trx, _try_, _trz);
                }

                WriteTargetInputs(feedback.X, feedback.Y, feedback.Z,
                                  feedback.Rx, feedback.Ry, feedback.Rz);

                SetStatus("目标看门狗已触发：已切换为保持当前位置并重新同步目标。", StatusWarnBrush);
            });
        }

        private void OnTargetResynced(RobotState state)
        {
            PostToUi(() =>
            {
                lock (_stateLock)
                {
                    _tx = state.X; _ty = state.Y; _tz = state.Z;
                    _trx = state.Rx; _try_ = state.Ry; _trz = state.Rz;
                }

                WriteTargetInputs(state.X, state.Y, state.Z,
                                  state.Rx, state.Ry, state.Rz);

                Log($"目标已同步到反馈位姿：X={Fmt(state.X)}, Y={Fmt(state.Y)}, Z={Fmt(state.Z)}", InfoBrush);
            });
        }

        private void OnParseError(Exception ex) => ReportError("解析错误", ex);

        private void OnSendError(Exception ex) => ReportError("发送错误", ex);

        // -----------------------------------------------------------------
        // 定时刷新 UI
        // -----------------------------------------------------------------
        private void OnUiTimerTick(object? sender, EventArgs e)
        {
            EgmClient? egm = _egm;
            if (egm is null)
            {
                UpdateSessionVisuals(EgmState.Disconnected);
                return;
            }

            UpdateSessionVisuals(egm.SessionState);

            int ageMs = egm.LastFeedbackAgeMs;
            if (ageMs < 0)
            {
                FeedbackAgeText.Text = "—";
                FeedbackAgeText.Foreground = MutedBrush;
            }
            else
            {
                FeedbackAgeText.Text = ageMs.ToString(CultureInfo.InvariantCulture) + " ms";
                FeedbackAgeText.Foreground = ageMs > EgmClient.FeedbackTimeoutMs
                    ? ErrorBrush
                    : OkBrush;
            }

            RemoteEndPointText.Text = "远端：" + (egm.RemoteEndPoint?.ToString() ?? "—");
            FrameCountText.Text = egm.FrameCount.ToString("N0", CultureInfo.InvariantCulture);

            RobotState state;
            double tx, ty, tz, trx, try_, trz;
            bool hasState;

            lock (_stateLock)
            {
                hasState = _hasLastState;
                state = _lastState;
                tx = _tx; ty = _ty; tz = _tz;
                trx = _trx; try_ = _try_; trz = _trz;
            }

            if (!hasState)
            {
                return;
            }

            if (!_targetInputsInitialized)
            {
                WriteTargetInputs(state.X, state.Y, state.Z,
                                  state.Rx, state.Ry, state.Rz);
                _targetInputsInitialized = true;
            }

            FbXText.Text = Fmt(state.X);
            FbYText.Text = Fmt(state.Y);
            FbZText.Text = Fmt(state.Z);
            FbRxText.Text = Fmt(state.Rx);
            FbRyText.Text = Fmt(state.Ry);
            FbRzText.Text = Fmt(state.Rz);

            TgXText.Text = Fmt(tx);
            TgYText.Text = Fmt(ty);
            TgZText.Text = Fmt(tz);
            TgRxText.Text = Fmt(trx);
            TgRyText.Text = Fmt(try_);
            TgRzText.Text = Fmt(trz);

            MciStateText.Text = state.MciState;
            MciStateText.Foreground = state.MciState.Contains("ERROR", StringComparison.OrdinalIgnoreCase)
                ? ErrorBrush
                : TextBrush;

            if (state.Converged is bool converged)
            {
                ConvergedText.Text = converged ? "是" : "否";
                ConvergedText.Foreground = converged ? OkBrush : MutedBrush;
            }
            else
            {
                ConvergedText.Text = "—";
                ConvergedText.Foreground = MutedBrush;
            }

            Joint1Text.Text = "J1  " + FormatJoint(state.Joints, 0);
            Joint2Text.Text = "J2  " + FormatJoint(state.Joints, 1);
            Joint3Text.Text = "J3  " + FormatJoint(state.Joints, 2);
            Joint4Text.Text = "J4  " + FormatJoint(state.Joints, 3);
            Joint5Text.Text = "J5  " + FormatJoint(state.Joints, 4);
            Joint6Text.Text = "J6  " + FormatJoint(state.Joints, 5);

            double deltaPosition = Distance(tx - state.X, ty - state.Y, tz - state.Z);
            double deltaRotation = MaxRotationDelta(
                trx, try_, trz,
                state.Rx, state.Ry, state.Rz);

            DeltaText.Text = $"Δ位置 {deltaPosition:F2} mm    Δ姿态 {deltaRotation:F2}°";
            DeltaText.Foreground = deltaPosition > MaxTranslationDeviationMm
                                   || deltaRotation > MaxRotationDeviationDeg
                ? ErrorBrush
                : TextBrush;

            if (egm.IsTargetExpired)
            {
                TargetLeaseText.Text = "目标流：看门狗已触发，保持当前位置";
                TargetLeaseText.Foreground = WarnBrush;
            }
            else
            {
                TargetLeaseText.Text = "目标流：活动（UI 心跳 100 ms / 看门狗 500 ms）";
                TargetLeaseText.Foreground = OkBrush;
            }
        }

        private void OnLeaseTimerTick(object? sender, EventArgs e)
        {
            EgmClient? egm = _egm;
            if (egm is null)
            {
                return;
            }

            // UI 线程存活时不断续租；若 UI 卡死，发送线程将在 500 ms 后保持当前位置。
            if (!egm.HasTarget || egm.IsTargetExpired)
            {
                return;
            }

            egm.RefreshTargetLease();
        }

        // -----------------------------------------------------------------
        // 目标更新 / 安全限幅
        // -----------------------------------------------------------------
        private void Move(
            double dx, double dy, double dz,
            double drx, double dry, double drz)
        {
            EgmClient? egm = _egm;
            if (egm is null)
            {
                return;
            }

            if (egm.SessionState != EgmState.Running)
            {
                SetStatus("EGM 非 RUNNING：移动命令已禁用。", StatusWarnBrush);
                return;
            }

            double x, y, z, rx, ry, rz;
            lock (_stateLock)
            {
                x = _tx + dx; y = _ty + dy; z = _tz + dz;
                rx = _trx + drx; ry = _try_ + dry; rz = _trz + drz;
            }

            if (!TrySetTarget(x, y, z, rx, ry, rz, out string reason))
            {
                Log("点动被拒绝：" + reason, WarnBrush);
                SetStatus("点动被拒绝：" + reason, StatusWarnBrush);
                return;
            }

            WriteTargetInputs(x, y, z, rx, ry, rz);
            SetStatus(
                $"目标已更新：X={Fmt(x)}, Y={Fmt(y)}, Z={Fmt(z)}, " +
                $"Rx={Fmt(rx)}, Ry={Fmt(ry)}, Rz={Fmt(rz)}",
                StatusInfoBrush);
        }

        private void ApplyTarget_Click(object sender, RoutedEventArgs e)
        {
            if (!TryReadTargetInputs(
                    out double x, out double y, out double z,
                    out double rx, out double ry, out double rz,
                    out string parseError))
            {
                Log("目标输入无效：" + parseError, ErrorBrush);
                SetStatus("目标输入无效：" + parseError, StatusErrorBrush);
                return;
            }

            if (!TrySetTarget(x, y, z, rx, ry, rz, out string reason))
            {
                Log("目标被拒绝：" + reason, WarnBrush);
                SetStatus("目标被拒绝：" + reason, StatusWarnBrush);
                return;
            }

            WriteTargetInputs(x, y, z, rx, ry, rz);
            Log(
                $"目标已下发：X={Fmt(x)}, Y={Fmt(y)}, Z={Fmt(z)}, " +
                $"Rx={Fmt(rx)}, Ry={Fmt(ry)}, Rz={Fmt(rz)}",
                InfoBrush);
            SetStatus("目标已下发，等待机器人收敛。", StatusOkBrush);
        }

        private void SyncTarget_Click(object sender, RoutedEventArgs e)
        {
            EgmClient? egm = _egm;
            if (egm is null || !egm.TryGetLatestFeedback(out RobotState feedback))
            {
                SetStatus("尚未收到机器人反馈，无法读取。", StatusWarnBrush);
                return;
            }

            WriteTargetInputs(feedback.X, feedback.Y, feedback.Z,
                              feedback.Rx, feedback.Ry, feedback.Rz);
            SetStatus("已读取当前反馈位姿到输入框（尚未下发）。", StatusInfoBrush);
        }

        private bool TrySetTarget(
            double x, double y, double z,
            double rx, double ry, double rz,
            out string reason)
        {
            reason = string.Empty;

            if (!double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(z)
                || !double.IsFinite(rx) || !double.IsFinite(ry) || !double.IsFinite(rz))
            {
                reason = "目标包含非有限数值。";
                return false;
            }

            EgmClient? egm = _egm;
            if (egm is null)
            {
                reason = "EGM 客户端未启动。";
                return false;
            }

            if (!egm.IsRobotConnected)
            {
                reason = "EGM 未连接或反馈已超时。";
                return false;
            }

            if (egm.SessionState != EgmState.Running)
            {
                reason = "EGM 非 RUNNING，移动已禁用。";
                return false;
            }

            lock (_stateLock)
            {
                if (!_hasLastState)
                {
                    reason = "尚未收到机器人反馈。";
                    return false;
                }

                RobotState feedback = _lastState;

                double positionDeviation = Distance(
                    x - feedback.X,
                    y - feedback.Y,
                    z - feedback.Z);

                double rotationDeviation = MaxRotationDelta(
                    rx, ry, rz,
                    feedback.Rx, feedback.Ry, feedback.Rz);

                if (positionDeviation > MaxTranslationDeviationMm + 1e-9)
                {
                    reason = $"平移偏差 {positionDeviation:F1} mm 超过 {MaxTranslationDeviationMm:F0} mm 限幅。";
                    return false;
                }

                if (rotationDeviation > MaxRotationDeviationDeg + 1e-9)
                {
                    reason = $"姿态偏差 {rotationDeviation:F1}° 超过 {MaxRotationDeviationDeg:F0}° 限幅。";
                    return false;
                }

                _tx = x; _ty = y; _tz = z;
                _trx = rx; _try_ = ry; _trz = rz;

                // 只更新目标；后台发送循环会以固定周期发送最新目标。
                egm.SetTarget(_tx, _ty, _tz, _trx, _try_, _trz);
            }

            return true;
        }

        private bool TryReadTargetInputs(
            out double x, out double y, out double z,
            out double rx, out double ry, out double rz,
            out string error)
        {
            x = y = z = rx = ry = rz = 0;
            error = string.Empty;

            if (!TryParseInput(TargetXInput.Text, "X", out x, out error)
                || !TryParseInput(TargetYInput.Text, "Y", out y, out error)
                || !TryParseInput(TargetZInput.Text, "Z", out z, out error)
                || !TryParseInput(TargetRxInput.Text, "Rx", out rx, out error)
                || !TryParseInput(TargetRyInput.Text, "Ry", out ry, out error)
                || !TryParseInput(TargetRzInput.Text, "Rz", out rz, out error))
            {
                return false;
            }

            return true;
        }

        private static bool TryParseInput(string text, string name, out double value, out string error)
        {
            if (!double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value)
                || !double.IsFinite(value))
            {
                error = $"{name} 必须是有效数字。";
                return false;
            }

            error = string.Empty;
            return true;
        }

        // 更新目标输入框的值
        private void WriteTargetInputs(
            double x, double y, double z,
            double rx, double ry, double rz)
        {
            // 用户正在编辑输入框时不要覆盖，避免抢走光标/输入内容。
            if (TargetXInput.IsKeyboardFocusWithin
                || TargetYInput.IsKeyboardFocusWithin
                || TargetZInput.IsKeyboardFocusWithin
                || TargetRxInput.IsKeyboardFocusWithin
                || TargetRyInput.IsKeyboardFocusWithin
                || TargetRzInput.IsKeyboardFocusWithin)
            {
                return;
            }

            TargetXInput.Text = Fmt(x);
            TargetYInput.Text = Fmt(y);
            TargetZInput.Text = Fmt(z);
            TargetRxInput.Text = Fmt(rx);
            TargetRyInput.Text = Fmt(ry);
            TargetRzInput.Text = Fmt(rz);
        }

        // -----------------------------------------------------------------
        // 会话 / 日志 / 状态栏
        // -----------------------------------------------------------------
        private void UpdateSessionVisuals(EgmState state)
        {
            SessionStateText.Text = state switch
            {
                EgmState.Running => "RUNNING",
                EgmState.Connected => "CONNECTED",
                _ => "DISCONNECTED"
            };

            SessionStateBadge.Background = state switch
            {
                EgmState.Running => StateRunningBrush,
                EgmState.Connected => StateConnectedBrush,
                _ => StateDisconnectedBrush
            };

            SessionStateDetailText.Text = state switch
            {
                EgmState.Running => "EGM 运行中：可点动或应用目标位姿。",
                EgmState.Connected => "通信正常，等待控制器进入 RUNNING；移动暂不可用。",
                _ => "反馈超时或尚未连接；发送已暂停，移动已禁用。"
            };
            SessionStateDetailText.Foreground = state == EgmState.Disconnected
                ? ErrorBrush
                : MutedBrush;

            bool running = state == EgmState.Running;
            JogPanel.IsEnabled = running;
            ApplyTargetButton.IsEnabled = running;
            SyncTargetButton.IsEnabled = state != EgmState.Disconnected;
        }

        private void ReportError(string source, Exception ex)
        {
            string signature = source + "|" + ex.GetType().FullName + "|" + ex.Message;
            DateTime now = DateTime.UtcNow;

            lock (_errorLock)
            {
                // 高频发送/解析错误会以 kHz 级刷新，2 秒内相同错误只记录一次。
                if (signature == _lastErrorSignature
                    && (now - _lastErrorUtc).TotalSeconds < 2.0)
                {
                    _lastErrorUtc = now;
                    return;
                }

                _lastErrorSignature = signature;
                _lastErrorUtc = now;
            }

            PostToUi(() => Log($"{source}：{ex.Message}", ErrorBrush));
        }

        private void Log(string message, Brush foreground)
        {
            if (_closing || Dispatcher.HasShutdownStarted)
            {
                return;
            }

            if (!Dispatcher.CheckAccess())
            {
                PostToUi(() => Log(message, foreground));
                return;
            }

            var entry = new LogEntry(
                DateTime.Now.ToString("HH:mm:ss.fff", CultureInfo.InvariantCulture) + "  " + message,
                foreground);

            LogList.Items.Add(entry);
            while (LogList.Items.Count > 500)
            {
                LogList.Items.RemoveAt(0);
            }

            LogList.ScrollIntoView(entry);
        }

        private void SetStatus(string message, Brush foreground)
        {
            if (!Dispatcher.CheckAccess())
            {
                PostToUi(() => SetStatus(message, foreground));
                return;
            }

            StatusText.Text = message;
            StatusText.Foreground = foreground;
        }

        private void PostToUi(Action action)
        {
            if (_closing || Dispatcher.HasShutdownStarted || Dispatcher.HasShutdownFinished)
            {
                return;
            }

            Dispatcher.BeginInvoke(DispatcherPriority.Normal, action);
        }

        private void ClearLog_Click(object sender, RoutedEventArgs e)
        {
            LogList.Items.Clear();
        }

        // -----------------------------------------------------------------
        // 窗口关闭
        // -----------------------------------------------------------------
        protected override void OnClosing(CancelEventArgs e)
        {
            _closing = true;
            _uiTimer.Stop();
            _leaseTimer.Stop();

            EgmClient? egm = _egm;
            _egm = null;

            if (egm != null)
            {
                egm.StateReceived -= OnRobotStateReceived;
                egm.SessionStateChanged -= OnSessionStateChanged;
                egm.TargetExpired -= OnTargetExpired;
                egm.TargetResynced -= OnTargetResynced;
                egm.ParseError -= OnParseError;
                egm.SendError -= OnSendError;
                egm.Dispose();
            }

            base.OnClosing(e);
        }

        // =================================================================
        // 平移按钮
        // =================================================================
        private void Translation_LeftButton_Click(object sender, RoutedEventArgs e)
            => Move(0, +TranslationIncrement, 0, 0, 0, 0);

        private void Translation_RightButton_Click(object sender, RoutedEventArgs e)
            => Move(0, -TranslationIncrement, 0, 0, 0, 0);

        private void Translation_UpButton_Click(object sender, RoutedEventArgs e)
            => Move(0, 0, +TranslationIncrement, 0, 0, 0);

        private void Translation_DownButton_Click(object sender, RoutedEventArgs e)
            => Move(0, 0, -TranslationIncrement, 0, 0, 0);

        private void Translation_ForwardButton_Click(object sender, RoutedEventArgs e)
            => Move(+TranslationIncrement, 0, 0, 0, 0, 0);

        private void Translation_BackwardButton_Click(object sender, RoutedEventArgs e)
            => Move(-TranslationIncrement, 0, 0, 0, 0, 0);

        // =================================================================
        // 旋转按钮
        // =================================================================
        private void Rotation_LeftButton_Click(object sender, RoutedEventArgs e)
            => Move(0, 0, 0, 0, -RotationIncrement, 0); // 绕 Y

        private void Rotation_RightButton_Click(object sender, RoutedEventArgs e)
            => Move(0, 0, 0, 0, +RotationIncrement, 0); // 绕 Y

        private void Rotation_UpButton_Click(object sender, RoutedEventArgs e)
            => Move(0, 0, 0, 0, 0, +RotationIncrement); // 绕 Z

        private void Rotation_DownButton_Click(object sender, RoutedEventArgs e)
            => Move(0, 0, 0, 0, 0, -RotationIncrement); // 绕 Z

        private void Rotation_ForwardButton_Click(object sender, RoutedEventArgs e)
            => Move(0, 0, 0, +RotationIncrement, 0, 0); // 绕 X

        private void Rotation_BackwardButton_Click(object sender, RoutedEventArgs e)
            => Move(0, 0, 0, -RotationIncrement, 0, 0); // 绕 X

        // =================================================================
        // 工具方法
        // =================================================================
        private static Brush Frozen(byte r, byte g, byte b)
        {
            var brush = new SolidColorBrush(Color.FromRgb(r, g, b));
            brush.Freeze();
            return brush;
        }

        private static string Fmt(double value)
            => value.ToString("F2", CultureInfo.InvariantCulture);

        private static string FormatJoint(double[]? joints, int index)
        {
            if (joints is null || index < 0 || index >= joints.Length)
            {
                return "—";
            }

            return joints[index].ToString("F2", CultureInfo.InvariantCulture) + "°";
        }

        private static double Distance(double dx, double dy, double dz)
            => Math.Sqrt(dx * dx + dy * dy + dz * dz);

        private static double MaxRotationDelta(
            double ax, double ay, double az,
            double bx, double by, double bz)
        {
            double dx = Math.Abs(NormalizeAngle(ax - bx));
            double dy = Math.Abs(NormalizeAngle(ay - by));
            double dz = Math.Abs(NormalizeAngle(az - bz));

            return Math.Max(dx, Math.Max(dy, dz));
        }

        private static double NormalizeAngle(double angle)
        {
            angle %= 360.0;

            if (angle > 180.0)
            {
                angle -= 360.0;
            }
            else if (angle < -180.0)
            {
                angle += 360.0;
            }

            return angle;
        }
    }

    /// <summary>日志列表项（UI 绑定 Text / Foreground）。</summary>
    public sealed class LogEntry
    {
        public LogEntry(string text, Brush foreground)
        {
            Text = text;
            Foreground = foreground;
        }

        public string Text { get; }

        public Brush Foreground { get; }
    }
}
