using System;
using System.Globalization;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Racer3MotionUi.Models;
using Racer3MotionUi.Services;

namespace Racer3MotionUi.ViewModels;

public sealed class MainViewModel : ObservableObject
{
    private const double MinVelocity = 0.005;
    private const double MaxVelocity = 0.10;
    private const double VelocityStep = 0.005;
    private const double MinShapeSizeMeters = 0.005;
    private const double MaxShapeSizeMeters = 0.15;
    private const double BiggerScale = 1.20;
    private const double SmallerScale = 0.80;
    private const double CenterStepMeters = 0.01;
    private static readonly ShapeKind DefaultShape = ShapeKind.Circle;

    private readonly IShapePathPlanner _shapePathPlanner;
    private readonly IRobotMotionService _robotMotionService;
    private readonly double _defaultVelocity;
    private readonly double _defaultShapeSizeMeters;
    private readonly CartesianPose _defaultCenter;
    private CancellationTokenSource? _runCancellation;
    private ShapeKind _selectedShape = ShapeKind.Circle;
    private double _velocity;
    private double _shapeSizeMeters;
    private double _centerX;
    private double _centerY;
    private double _centerZ;
    private bool _isDryRun = true;
    private bool _confirmMotion;
    private bool _isProcessActive;
    private string _modeState = "Dry run ready";
    private string _executionSummary = string.Empty;
    private string _lastRunStatus = "No run yet";
    private string _lastRunDetail = "Select a shape and run controller validation first.";
    private string _commandPreview = string.Empty;
    private string _waypointPreview = string.Empty;
    private string _processLog = string.Empty;
    private string _motionAssistantInput = string.Empty;
    private string _motionAssistantLog = string.Empty;

    public MainViewModel(
        IShapePathPlanner shapePathPlanner,
        IRobotMotionService robotMotionService,
        Racer3MotionUiConfig config)
    {
        _shapePathPlanner = shapePathPlanner;
        _robotMotionService = robotMotionService;
        _defaultVelocity = config.DefaultVelocity;
        _defaultShapeSizeMeters = config.DefaultShapeSizeMeters;
        _defaultCenter = config.DefaultCenter;
        _velocity = config.DefaultVelocity;
        _shapeSizeMeters = config.DefaultShapeSizeMeters;
        _centerX = config.DefaultCenter.X;
        _centerY = config.DefaultCenter.Y;
        _centerZ = config.DefaultCenter.Z;

        SelectShapeCommand = new RelayCommand<string?>(SelectShape);
        InterpretMotionCommandCommand = new RelayCommand(InterpretMotionCommand);
        RunSelectedShapeCommand = new AsyncRelayCommand(RunSelectedShapeAsync, CanRunSelectedShape);
        StopCommand = new RelayCommand(StopProcess, () => IsProcessActive);
        ClearLogCommand = new RelayCommand(() => ProcessLog = string.Empty);

        AppendAssistantLog("Assistant ready. Commands update the plan preview only.");
        RefreshPlanAndPreview();
    }

    public IRelayCommand<string?> SelectShapeCommand { get; }

    public IRelayCommand InterpretMotionCommandCommand { get; }

    public IAsyncRelayCommand RunSelectedShapeCommand { get; }

    public IRelayCommand StopCommand { get; }

    public IRelayCommand ClearLogCommand { get; }

    public string Title => "Racer3 Shape Trace Demo";

    public string ConnectionState => "Local PowerShell launcher";

    public ShapeKind SelectedShape
    {
        get => _selectedShape;
        private set
        {
            if (SetProperty(ref _selectedShape, value))
            {
                OnPropertyChanged(nameof(SelectedShapeName));
                OnPropertyChanged(nameof(IsCircleSelected));
                OnPropertyChanged(nameof(IsSquareSelected));
                OnPropertyChanged(nameof(IsTriangleSelected));
                OnPropertyChanged(nameof(IsHexagonSelected));
                RefreshPlanAndPreview();
            }
        }
    }

    public string SelectedShapeName => SelectedShape.ToString();

    public bool IsCircleSelected => SelectedShape == ShapeKind.Circle;

    public bool IsSquareSelected => SelectedShape == ShapeKind.Square;

    public bool IsTriangleSelected => SelectedShape == ShapeKind.Triangle;

    public bool IsHexagonSelected => SelectedShape == ShapeKind.Hexagon;

    public double Velocity
    {
        get => _velocity;
        set
        {
            if (SetProperty(ref _velocity, value))
            {
                RefreshPlanAndPreview();
            }
        }
    }

    public double ShapeSizeMeters
    {
        get => _shapeSizeMeters;
        set
        {
            if (SetProperty(ref _shapeSizeMeters, value))
            {
                RefreshPlanAndPreview();
            }
        }
    }

    public double CenterX
    {
        get => _centerX;
        set
        {
            if (SetProperty(ref _centerX, value))
            {
                RefreshPlanAndPreview();
            }
        }
    }

    public double CenterY
    {
        get => _centerY;
        set
        {
            if (SetProperty(ref _centerY, value))
            {
                RefreshPlanAndPreview();
            }
        }
    }

    public double CenterZ
    {
        get => _centerZ;
        set
        {
            if (SetProperty(ref _centerZ, value))
            {
                RefreshPlanAndPreview();
            }
        }
    }

    public bool IsDryRun
    {
        get => _isDryRun;
        set
        {
            if (SetProperty(ref _isDryRun, value))
            {
                if (_isDryRun && ConfirmMotion)
                {
                    ConfirmMotion = false;
                }

                OnPropertyChanged(nameof(IsConfirmMotionEnabled));
                RefreshModeState();
                RefreshPlanAndPreview();
            }
        }
    }

    public bool ConfirmMotion
    {
        get => _confirmMotion;
        set
        {
            var requestedValue = value && IsConfirmMotionEnabled;
            if (SetProperty(ref _confirmMotion, requestedValue))
            {
                RefreshModeState();
                RefreshPlanAndPreview();
            }
        }
    }

    public bool IsConfirmMotionEnabled => !IsDryRun && !IsProcessActive;

    public bool IsProcessActive
    {
        get => _isProcessActive;
        private set
        {
            if (SetProperty(ref _isProcessActive, value))
            {
                OnPropertyChanged(nameof(IsConfirmMotionEnabled));
                if (IsProcessActive && ConfirmMotion)
                {
                    ConfirmMotion = false;
                }

                RunSelectedShapeCommand.NotifyCanExecuteChanged();
                StopCommand.NotifyCanExecuteChanged();
            }
        }
    }

    public string ModeState
    {
        get => _modeState;
        private set => SetProperty(ref _modeState, value);
    }

    public string CommandPreview
    {
        get => _commandPreview;
        private set => SetProperty(ref _commandPreview, value);
    }

    public string ExecutionSummary
    {
        get => _executionSummary;
        private set => SetProperty(ref _executionSummary, value);
    }

    public string LastRunStatus
    {
        get => _lastRunStatus;
        private set => SetProperty(ref _lastRunStatus, value);
    }

    public string LastRunDetail
    {
        get => _lastRunDetail;
        private set => SetProperty(ref _lastRunDetail, value);
    }

    public string WaypointPreview
    {
        get => _waypointPreview;
        private set => SetProperty(ref _waypointPreview, value);
    }

    public string ProcessLog
    {
        get => _processLog;
        private set => SetProperty(ref _processLog, value);
    }

    public string MotionAssistantInput
    {
        get => _motionAssistantInput;
        set => SetProperty(ref _motionAssistantInput, value);
    }

    public string MotionAssistantLog
    {
        get => _motionAssistantLog;
        private set => SetProperty(ref _motionAssistantLog, value);
    }

    private void SelectShape(string? shapeName)
    {
        if (Enum.TryParse<ShapeKind>(shapeName, out var shape))
        {
            SelectedShape = shape;
        }
    }

    private void InterpretMotionCommand()
    {
        var rawCommand = MotionAssistantInput.Trim();
        if (string.IsNullOrWhiteSpace(rawCommand))
        {
            AppendAssistantLog("Type a command such as draw square or move up.");
            return;
        }

        var normalizedCommand = NormalizeAssistantCommand(rawCommand);
        AppendAssistantLog($"> {rawCommand}");

        var response = normalizedCommand switch
        {
            "draw square" => SelectAssistantShape(ShapeKind.Square),
            "draw triangle" => SelectAssistantShape(ShapeKind.Triangle),
            "draw circle" => SelectAssistantShape(ShapeKind.Circle),
            "draw hexagon" => SelectAssistantShape(ShapeKind.Hexagon),
            "make it bigger" => ScaleAssistantShape(BiggerScale),
            "make it smaller" => ScaleAssistantShape(SmallerScale),
            "move up" => MoveAssistantCenter(deltaY: 0.0, deltaZ: CenterStepMeters),
            "move down" => MoveAssistantCenter(deltaY: 0.0, deltaZ: -CenterStepMeters),
            "move left" => MoveAssistantCenter(deltaY: -CenterStepMeters, deltaZ: 0.0),
            "move right" => MoveAssistantCenter(deltaY: CenterStepMeters, deltaZ: 0.0),
            "go faster" => AdjustAssistantVelocity(VelocityStep),
            "go slower" => AdjustAssistantVelocity(-VelocityStep),
            "reset" => ResetAssistantPlan(),
            "run it" => "Plan is ready. Validate first, then arm Confirm Motion and press Run Selected Shape.",
            _ => "I can interpret: draw square, draw triangle, draw circle, draw hexagon, make it bigger, make it smaller, move up, move down, move left, move right, go faster, go slower, reset, run it."
        };

        AppendAssistantLog(response);
        MotionAssistantInput = string.Empty;
    }

    private bool CanRunSelectedShape()
    {
        return !IsProcessActive &&
               Velocity > 0.0 &&
               ShapeSizeMeters > 0.0 &&
               (IsDryRun || ConfirmMotion);
    }

    private async Task RunSelectedShapeAsync()
    {
        var plan = CreateCurrentPlan();
        var options = CreateMotionOptions();
        LastRunStatus = options.ConfirmMotion ? "Live run starting" : "Validation starting";
        LastRunDetail = $"{plan.Shape}: {plan.Waypoints.Count} requested waypoint(s).";
        AppendLog("ui", $"Run requested for {plan.Shape}; dry run: {options.DryRun}; confirm motion: {options.ConfirmMotion}.");
        AppendLog("ui", "Command preview at launch:");
        AppendLog("cmd", CommandPreview.Replace(Environment.NewLine, " "));

        using var cancellation = new CancellationTokenSource();
        _runCancellation = cancellation;
        IsProcessActive = true;
        ModeState = "Running";

        try
        {
            var progress = new Progress<ProcessOutputLine>(line => AppendLog(line.Stream, line.Text));
            var result = await _robotMotionService.TraceShapeAsync(plan, options, progress, cancellation.Token);
            AppendLog("ui", $"Process exited with code {result.ExitCode}.");
            if (result.Succeeded)
            {
                LastRunStatus = options.ConfirmMotion ? "Live complete" : "Validated";
                LastRunDetail = options.ConfirmMotion
                    ? $"{plan.Shape}: backend completed live trace and returned to zero."
                    : $"{plan.Shape}: backend accepted {plan.Waypoints.Count} waypoint(s); no motion commanded.";
            }
            else
            {
                LastRunStatus = "Failed";
                LastRunDetail = $"{plan.Shape}: process exited with code {result.ExitCode}. Check Process Log.";
            }

            ModeState = result.Succeeded ? "Complete" : "Process failed";
        }
        catch (OperationCanceledException)
        {
            LastRunStatus = "Stopped";
            LastRunDetail = "Operator cancelled the active process.";
            AppendLog("ui", "Run stopped by operator.");
            ModeState = "Stopped";
        }
        catch (Exception exception)
        {
            LastRunStatus = "Error";
            LastRunDetail = exception.Message;
            AppendLog("err", exception.Message);
            ModeState = "Error";
        }
        finally
        {
            _runCancellation = null;
            IsProcessActive = false;
            RefreshModeState();
        }
    }

    private void StopProcess()
    {
        _runCancellation?.Cancel();
    }

    private string SelectAssistantShape(ShapeKind shape)
    {
        SelectedShape = shape;
        return $"{shape} selected. Waypoint preview and command preview updated.";
    }

    private string ScaleAssistantShape(double scale)
    {
        ShapeSizeMeters = RoundMeters(Math.Clamp(
            ShapeSizeMeters * scale,
            MinShapeSizeMeters,
            MaxShapeSizeMeters));

        return FormattableString.Invariant(
            $"Size/radius set to {ShapeSizeMeters:0.0000} m. Waypoint preview updated.");
    }

    private string MoveAssistantCenter(double deltaY, double deltaZ)
    {
        CenterY = RoundMeters(CenterY + deltaY);
        CenterZ = RoundMeters(CenterZ + deltaZ);

        return FormattableString.Invariant(
            $"Center set to Y={CenterY:0.0000} m, Z={CenterZ:0.0000} m. Waypoint preview updated.");
    }

    private string AdjustAssistantVelocity(double delta)
    {
        Velocity = RoundVelocity(Math.Clamp(
            Velocity + delta,
            MinVelocity,
            MaxVelocity));

        return FormattableString.Invariant(
            $"Velocity set to {Velocity:0.0000} m/s. Command preview updated.");
    }

    private string ResetAssistantPlan()
    {
        SelectedShape = DefaultShape;
        Velocity = _defaultVelocity;
        ShapeSizeMeters = _defaultShapeSizeMeters;
        CenterX = _defaultCenter.X;
        CenterY = _defaultCenter.Y;
        CenterZ = _defaultCenter.Z;
        IsDryRun = true;
        ConfirmMotion = false;

        return "Reset to safe defaults. Dry Run / Validate Only is enabled and live motion is disarmed.";
    }

    private static string NormalizeAssistantCommand(string command)
    {
        var cleaned = command.Trim().Trim('.', '!', '?').ToLowerInvariant();
        return string.Join(' ', cleaned.Split(Array.Empty<char>(), StringSplitOptions.RemoveEmptyEntries));
    }

    private static double RoundMeters(double value)
    {
        return Math.Round(value, 4);
    }

    private static double RoundVelocity(double value)
    {
        return Math.Round(value, 4);
    }

    private ShapeTracePlan CreateCurrentPlan()
    {
        return _shapePathPlanner.CreatePlan(
            SelectedShape,
            new CartesianPose(CenterX, CenterY, CenterZ),
            ShapeSizeMeters);
    }

    private RobotMotionOptions CreateMotionOptions()
    {
        return new RobotMotionOptions
        {
            Velocity = Velocity,
            DryRun = IsDryRun,
            ConfirmMotion = ConfirmMotion
        };
    }

    private void RefreshPlanAndPreview()
    {
        try
        {
            var plan = CreateCurrentPlan();
            var options = CreateMotionOptions();
            var commands = _robotMotionService.BuildTraceCommands(plan, options);

            var preview = new StringBuilder();
            preview.AppendLine(IsDryRun
                ? $"{plan.Shape} waypoints; validation mode sends the full constant-X Y/Z-plane waypoint list to backend trace planning without -ConfirmMotion."
                : $"{plan.Shape} waypoints; live mode sends the full constant-X Y/Z-plane waypoint list and requires backend validation before motion.");
            preview.AppendLine(FormattableString.Invariant($"Plane: constant X={CenterX:0.0000}; center Y={CenterY:0.0000}, Z={CenterZ:0.0000}; size/radius={ShapeSizeMeters:0.0000} m"));
            preview.AppendLine();
            foreach (var item in plan.Waypoints.Select((pose, index) => pose.ToPreviewLine(index + 1)))
            {
                preview.AppendLine(item);
            }

            WaypointPreview = preview.ToString();
            CommandPreview = BuildCommandPreview(commands);
            ExecutionSummary = IsDryRun
                ? $"Controller validation only: connects/reads RMP, checks {plan.Waypoints.Count} waypoint(s), omits -ConfirmMotion, and should not enable amps."
                : ConfirmMotion
                    ? $"LIVE MOTION ARMED: streams {plan.Waypoints.Count} waypoint(s) only after backend cartesian-trace validation gates pass."
                    : "Live mode is locked. Check Confirm Motion to arm live movement, or check Dry Run / Validate Only to validate without motion.";
        }
        catch (Exception exception)
        {
            WaypointPreview = exception.Message;
            CommandPreview = "No runnable command until the plan inputs are valid.";
            ExecutionSummary = "Fix the plan inputs before running.";
        }

        RunSelectedShapeCommand.NotifyCanExecuteChanged();
    }

    private void RefreshModeState()
    {
        if (IsProcessActive)
        {
            ModeState = "Running";
            return;
        }

        ModeState = IsDryRun
            ? "Controller validation ready"
            : ConfirmMotion
                ? "LIVE MOTION ARMED"
                : "Live motion locked";
    }

    private static string BuildCommandPreview(IReadOnlyList<MotionCommand> commands)
    {
        var builder = new StringBuilder();
        for (var index = 0; index < commands.Count; index++)
        {
            if (index > 0)
            {
                builder.AppendLine();
            }

            builder.AppendLine($"# Trace command {index + 1} of {commands.Count}");
            builder.AppendLine(commands[index].DisplayText);
        }

        return builder.ToString();
    }

    private void AppendLog(string stream, string text)
    {
        var timestamp = DateTime.Now.ToString("HH:mm:ss", CultureInfo.InvariantCulture);
        ProcessLog += $"[{timestamp}] [{stream}] {text}{Environment.NewLine}";
    }

    private void AppendAssistantLog(string text)
    {
        var timestamp = DateTime.Now.ToString("HH:mm:ss", CultureInfo.InvariantCulture);
        var entry = $"[{timestamp}] {text}";
        var nextLog = string.IsNullOrEmpty(MotionAssistantLog)
            ? entry
            : $"{MotionAssistantLog}{Environment.NewLine}{entry}";
        var lines = nextLog.Split(new[] { Environment.NewLine }, StringSplitOptions.None);

        MotionAssistantLog = string.Join(
            Environment.NewLine,
            lines.Skip(Math.Max(0, lines.Length - 24)));
    }
}
