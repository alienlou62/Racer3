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
    private readonly IShapePathPlanner _shapePathPlanner;
    private readonly IRobotMotionService _robotMotionService;
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
    private string _commandPreview = string.Empty;
    private string _waypointPreview = string.Empty;
    private string _processLog = string.Empty;

    public MainViewModel(
        IShapePathPlanner shapePathPlanner,
        IRobotMotionService robotMotionService,
        Racer3MotionUiConfig config)
    {
        _shapePathPlanner = shapePathPlanner;
        _robotMotionService = robotMotionService;
        _velocity = config.DefaultVelocity;
        _shapeSizeMeters = config.DefaultShapeSizeMeters;
        _centerX = config.DefaultCenter.X;
        _centerY = config.DefaultCenter.Y;
        _centerZ = config.DefaultCenter.Z;

        SelectShapeCommand = new RelayCommand<string?>(SelectShape);
        RunSelectedShapeCommand = new AsyncRelayCommand(RunSelectedShapeAsync, CanRunSelectedShape);
        StopCommand = new RelayCommand(StopProcess, () => IsProcessActive);
        ClearLogCommand = new RelayCommand(() => ProcessLog = string.Empty);

        RefreshPlanAndPreview();
    }

    public IRelayCommand<string?> SelectShapeCommand { get; }

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
            if (SetProperty(ref _confirmMotion, value))
            {
                RefreshModeState();
                RefreshPlanAndPreview();
            }
        }
    }

    public bool IsProcessActive
    {
        get => _isProcessActive;
        private set
        {
            if (SetProperty(ref _isProcessActive, value))
            {
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

    private void SelectShape(string? shapeName)
    {
        if (Enum.TryParse<ShapeKind>(shapeName, out var shape))
        {
            SelectedShape = shape;
        }
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
            ModeState = result.Succeeded ? "Complete" : "Process failed";
        }
        catch (OperationCanceledException)
        {
            AppendLog("ui", "Run stopped by operator.");
            ModeState = "Stopped";
        }
        catch (Exception exception)
        {
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
                ? $"{plan.Shape} waypoints; validation mode sends the full waypoint list to backend trace planning without -ConfirmMotion."
                : $"{plan.Shape} waypoints; live mode sends the full waypoint list and requires backend validation before motion.");
            preview.AppendLine(FormattableString.Invariant($"Center: X={CenterX:0.0000}, Y={CenterY:0.0000}, Z={CenterZ:0.0000}; size/radius={ShapeSizeMeters:0.0000} m"));
            preview.AppendLine();
            foreach (var item in plan.Waypoints.Select((pose, index) => pose.ToPreviewLine(index + 1)))
            {
                preview.AppendLine(item);
            }

            WaypointPreview = preview.ToString();
            CommandPreview = BuildCommandPreview(commands);
            ExecutionSummary = IsDryRun
                ? $"Validate only: connects/reads RMP, checks {plan.Waypoints.Count} waypoint(s), omits -ConfirmMotion, and should not enable amps."
                : $"Live mode streams {plan.Waypoints.Count} waypoint(s) through backend cartesian-trace after validation gates pass.";
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
            ? "Dry run ready"
            : ConfirmMotion
                ? "Live motion armed"
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
}
