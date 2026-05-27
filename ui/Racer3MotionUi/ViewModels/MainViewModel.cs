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
    private const double MinCenterX = 0.40;
    private const double MaxCenterX = 0.60;
    private const double MinCenterY = -0.15;
    private const double MaxCenterY = 0.15;
    private const double MinCenterZ = -0.65;
    private const double MaxCenterZ = -0.45;
    private const double MinVelocity = 0.02;
    private const double MaxVelocity = 0.08;
    private const double MinShapeSizeMeters = 0.02;
    private const double MaxShapeSizeMeters = 0.08;
    private static readonly ShapeKind DefaultShape = ShapeKind.Circle;

    private readonly IShapePathPlanner _shapePathPlanner;
    private readonly IRobotMotionService _robotMotionService;
    private readonly IRobotSessionService _robotSessionService;
    private readonly IMotionChatService _llmMotionChatService;
    private readonly IMotionChatService _localMotionChatService;
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
    private string _motionAssistantStatus = string.Empty;
    private bool _useLlmAssistant = true;
    private bool _isMotionAssistantBusy;
    private string _robotSessionStatus = "Not connected - armed session ready to start";
    private bool _keepAmpsEnabledDuringSession = true;
    private bool _holdFinalPoseAfterCommand = true;
    private bool _returnToZeroAfterCommand;
    private bool _isJogModeEnabled;
    private double _jogStepMeters = 0.005;
    private double _jogVelocity = 0.003;
    private bool _isBackendJogActive;
    private string _activeJogDirection = string.Empty;
    private readonly SemaphoreSlim _jogCommandSemaphore = new(1, 1);

    public MainViewModel(
        IShapePathPlanner shapePathPlanner,
        IRobotMotionService robotMotionService,
        IRobotSessionService robotSessionService,
        IMotionChatService llmMotionChatService,
        IMotionChatService localMotionChatService,
        Racer3MotionUiConfig config)
    {
        _shapePathPlanner = shapePathPlanner;
        _robotMotionService = robotMotionService;
        _robotSessionService = robotSessionService;
        _llmMotionChatService = llmMotionChatService;
        _localMotionChatService = localMotionChatService;
        _defaultVelocity = config.DefaultVelocity;
        _defaultShapeSizeMeters = config.DefaultShapeSizeMeters;
        _defaultCenter = config.DefaultCenter;
        _velocity = config.DefaultVelocity;
        _shapeSizeMeters = config.DefaultShapeSizeMeters;
        _centerX = config.DefaultCenter.X;
        _centerY = config.DefaultCenter.Y;
        _centerZ = config.DefaultCenter.Z;

        SelectShapeCommand = new RelayCommand<string?>(SelectShape);
        InterpretMotionCommandCommand = new AsyncRelayCommand(InterpretMotionCommandAsync);
        RunSelectedShapeCommand = new AsyncRelayCommand(RunSelectedShapeAsync, CanRunSelectedShape);
        StopCommand = new RelayCommand(StopProcess, () => IsProcessActive);
        ClearLogCommand = new RelayCommand(() => ProcessLog = string.Empty);
        StartArmedSessionCommand = new AsyncRelayCommand(StartArmedSessionAsync, CanStartArmedSession);
        ShutdownSessionCommand = new AsyncRelayCommand(ShutdownSessionAsync, CanUseSessionControls);
        StopSessionMotionCommand = new AsyncRelayCommand(StopSessionMotionAsync, CanUseSessionControls);
        RunSessionShapeCommand = new AsyncRelayCommand(RunSessionShapeAsync, CanRunSessionShape);

        RefreshMotionAssistantStatus();
        AppendAssistantLog("Assistant ready. Chat updates the plan preview only.");
        RefreshPlanAndPreview();
    }

    public IRelayCommand<string?> SelectShapeCommand { get; }

    public IAsyncRelayCommand InterpretMotionCommandCommand { get; }

    public IAsyncRelayCommand RunSelectedShapeCommand { get; }

    public IRelayCommand StopCommand { get; }

    public IRelayCommand ClearLogCommand { get; }

    public IAsyncRelayCommand StartArmedSessionCommand { get; }

    public IAsyncRelayCommand ShutdownSessionCommand { get; }

    public IAsyncRelayCommand StopSessionMotionCommand { get; }

    public IAsyncRelayCommand RunSessionShapeCommand { get; }

    public bool CanUseBackendJogControls => IsJogModeEnabled && _robotSessionService.IsRunning && !IsProcessActive;

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
                OnPropertyChanged(nameof(CanUseBackendJogControls));
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
                OnPropertyChanged(nameof(CanUseBackendJogControls));
                if (IsProcessActive && ConfirmMotion)
                {
                    ConfirmMotion = false;
                }

                RunSelectedShapeCommand.NotifyCanExecuteChanged();
                StopCommand.NotifyCanExecuteChanged();
                StartArmedSessionCommand.NotifyCanExecuteChanged();
                RunSessionShapeCommand.NotifyCanExecuteChanged();
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

    public string MotionAssistantStatus
    {
        get => _motionAssistantStatus;
        private set => SetProperty(ref _motionAssistantStatus, value);
    }

    public bool UseLlmAssistant
    {
        get => _useLlmAssistant;
        set
        {
            if (SetProperty(ref _useLlmAssistant, value))
            {
                RefreshMotionAssistantStatus();
            }
        }
    }

    public bool IsMotionAssistantBusy
    {
        get => _isMotionAssistantBusy;
        private set => SetProperty(ref _isMotionAssistantBusy, value);
    }

    public string RobotSessionStatus
    {
        get => _robotSessionStatus;
        private set => SetProperty(ref _robotSessionStatus, value);
    }

    public bool KeepAmpsEnabledDuringSession
    {
        get => _keepAmpsEnabledDuringSession;
        set => SetProperty(ref _keepAmpsEnabledDuringSession, value);
    }

    public bool HoldFinalPoseAfterCommand
    {
        get => _holdFinalPoseAfterCommand;
        set
        {
            if (SetProperty(ref _holdFinalPoseAfterCommand, value) && value && ReturnToZeroAfterCommand)
            {
                ReturnToZeroAfterCommand = false;
            }
        }
    }

    public bool ReturnToZeroAfterCommand
    {
        get => _returnToZeroAfterCommand;
        set
        {
            if (SetProperty(ref _returnToZeroAfterCommand, value) && value && HoldFinalPoseAfterCommand)
            {
                HoldFinalPoseAfterCommand = false;
            }
        }
    }

    public bool IsJogModeEnabled
    {
        get => _isJogModeEnabled;
        set
        {
            if (SetProperty(ref _isJogModeEnabled, value))
            {
                OnPropertyChanged(nameof(CanUseBackendJogControls));

                if (!value && IsBackendJogActive)
                {
                    _ = StopBackendCartesianJogAsync("jog mode disabled");
                }
            }
        }
    }

    public double JogStepMeters
    {
        get => _jogStepMeters;
        set => SetProperty(ref _jogStepMeters, Math.Clamp(value, 0.001, 0.025));
    }

    public double JogVelocity
    {
        get => _jogVelocity;
        set => SetProperty(ref _jogVelocity, Math.Clamp(value, 0.0001, 0.004));
    }

    public bool IsBackendJogActive
    {
        get => _isBackendJogActive;
        private set
        {
            if (SetProperty(ref _isBackendJogActive, value))
            {
                OnPropertyChanged(nameof(CanUseBackendJogControls));
                RunSessionShapeCommand.NotifyCanExecuteChanged();
            }
        }
    }

    public string ActiveJogDirection
    {
        get => _activeJogDirection;
        private set => SetProperty(ref _activeJogDirection, value);
    }

    private void SelectShape(string? shapeName)
    {
        if (Enum.TryParse<ShapeKind>(shapeName, out var shape))
        {
            SelectedShape = shape;
        }
    }

    private async Task InterpretMotionCommandAsync()
    {
        var rawCommand = MotionAssistantInput.Trim();
        if (string.IsNullOrWhiteSpace(rawCommand))
        {
            AppendAssistantLog("Type a command such as draw a bigger square higher up.");
            return;
        }

        AppendAssistantLog($"> {rawCommand}");

        if (RuleBasedMotionChatService.HasExecutionIntent(rawCommand))
        {
            RefreshMotionAssistantStatus();
            ApplyMotionChatResponse(new MotionChatResponse(
                MotionChatAction.Explain,
                "I prepared the plan. Validate first, then arm Confirm Motion and press Run Selected Shape."));
            MotionAssistantInput = string.Empty;
            return;
        }

        if (TryHandleAssistantMetaCommand(rawCommand, out var metaResponse))
        {
            RefreshMotionAssistantStatus();
            AppendAssistantLog(metaResponse);
            MotionAssistantInput = string.Empty;
            return;
        }

        IsMotionAssistantBusy = true;

        try
        {
            var request = new MotionChatRequest(
                rawCommand,
                CreateCurrentMotionPlan(),
                CreateDefaultMotionPlan());

            var service = SelectMotionChatService();
            MotionChatResponse response;
            if (UseLlmAssistant && !_llmMotionChatService.IsAvailable)
            {
                AppendAssistantLog(_llmMotionChatService.StatusText);
                response = await _localMotionChatService.InterpretAsync(request, CancellationToken.None);
            }
            else
            {
                response = await service.InterpretAsync(request, CancellationToken.None);
            }

            MotionAssistantStatus = service.StatusText;
            ApplyMotionChatResponse(response);
            MotionAssistantInput = string.Empty;
        }
        catch (MotionChatInvalidResponseException exception)
        {
            MotionAssistantStatus = "Error";
            AppendAssistantLog(exception.Message);
        }
        catch (MotionChatServiceException exception)
        {
            MotionAssistantStatus = "Error";
            AppendAssistantLog($"LLM planner error: {exception.Message} The current plan was not changed.");
        }
        catch (Exception exception)
        {
            MotionAssistantStatus = "Error";
            AppendAssistantLog($"Assistant error: {exception.Message} The current plan was not changed.");
        }
        finally
        {
            IsMotionAssistantBusy = false;
        }
    }

    private bool TryHandleAssistantMetaCommand(string rawCommand, out string response)
    {
        var command = NormalizeAssistantCommand(rawCommand);

        if (command is "help" or "what can you do" or "what do you do" or "commands")
        {
            response = "I can plan robot shape previews only. Try: draw square, draw a bigger triangle higher up, make it slower, move left, reset. I cannot execute motion from chat; validate and use Run Selected Shape manually.";
            return true;
        }

        if (command is "what are you" or "who are you" or "are you an ai" or "are you ai")
        {
            response = "I am the Racer3 Motion Assistant. I convert chat requests into safe preview plans for the shape trace UI. I cannot directly run the robot.";
            return true;
        }

        if (command is "what mode are you in"
            or "what provider are you using"
            or "which provider are you using"
            or "what model are you using"
            or "what model do you run"
            or "which model are you"
            or "what is your model"
            or "what is your provider")
        {
            response = BuildAssistantProviderStatusResponse();
            return true;
        }

        response = string.Empty;
        return false;
    }

    private string BuildAssistantProviderStatusResponse()
    {
        var statusText = UseLlmAssistant
            ? _llmMotionChatService.StatusText
            : _localMotionChatService.StatusText;

        var provider = UseLlmAssistant ? "configured assistant" : "LocalRules";
        var model = "none";
        var details = string.Empty;

        if (statusText.StartsWith("OpenAI mode:", StringComparison.OrdinalIgnoreCase))
        {
            provider = "OpenAI";
            model = statusText["OpenAI mode:".Length..].Trim();
        }
        else if (statusText.StartsWith("Gemini mode:", StringComparison.OrdinalIgnoreCase))
        {
            provider = "Gemini";
            model = statusText["Gemini mode:".Length..].Trim();
        }
        else if (statusText.StartsWith("Ollama mode:", StringComparison.OrdinalIgnoreCase))
        {
            provider = "Ollama";
            var ollamaStatus = statusText["Ollama mode:".Length..].Trim();
            var atIndex = ollamaStatus.IndexOf(" at ", StringComparison.OrdinalIgnoreCase);
            if (atIndex >= 0)
            {
                model = ollamaStatus[..atIndex].Trim();
                details = $" Base URL: {ollamaStatus[(atIndex + 4)..].Trim()}.";
            }
            else
            {
                model = ollamaStatus;
            }
        }
        else if (statusText.Contains("local", StringComparison.OrdinalIgnoreCase)
                 || statusText.Contains("rules", StringComparison.OrdinalIgnoreCase))
        {
            provider = "LocalRules";
            model = "none";
        }
        else
        {
            details = $" Status: {statusText}";
        }

        return $"Assistant provider: {provider}. Model: {model}.{details} Chat updates preview only.";
    }

    private static string NormalizeAssistantCommand(string input)
    {
        var builder = new StringBuilder(input.Length);
        foreach (var character in input.Trim().ToLowerInvariant())
        {
            builder.Append(char.IsLetterOrDigit(character) ? character : ' ');
        }

        return string.Join(
            ' ',
            builder
                .ToString()
                .Split(' ', StringSplitOptions.RemoveEmptyEntries));
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
        if (_robotSessionService.IsRunning)
        {
            LastRunStatus = "Blocked";
            LastRunDetail = "Persistent armed session is active. Use Run Shape In Session instead of the one-shot runner.";
            ModeState = "One-shot blocked";
            AppendLog("ui", "Run Selected Shape blocked because the persistent armed session is active. Use Run Shape In Session instead.");
            return;
        }

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

    private bool CanStartArmedSession()
    {
        return !IsProcessActive && !_robotSessionService.IsRunning;
    }

    private bool CanUseSessionControls()
    {
        return _robotSessionService.IsRunning;
    }

    private bool CanRunSessionShape()
    {
        return !IsProcessActive &&
               _robotSessionService.IsRunning &&
               !IsBackendJogActive &&
               !IsDryRun &&
               ConfirmMotion &&
               Velocity > 0.0 &&
               ShapeSizeMeters > 0.0;
    }

    private async Task RunSessionShapeAsync()
    {
        var plan = CreateCurrentPlan();
        var options = CreateMotionOptions();
        LastRunStatus = "Session trace starting";
        LastRunDetail = $"{plan.Shape}: {plan.Waypoints.Count} waypoint(s), amps remain enabled after trace.";
        RobotSessionStatus = "Session trace running...";
        AppendLog("ui", $"Session trace requested for {plan.Shape}; confirm motion: {options.ConfirmMotion}; persistent session running: {_robotSessionService.IsRunning}.");

        using var cancellation = new CancellationTokenSource();
        _runCancellation = cancellation;
        IsProcessActive = true;
        ModeState = "Session trace running";

        try
        {
            var progress = new Progress<ProcessOutputLine>(line => AppendLog(line.Stream, line.Text));
            var result = await _robotSessionService.TraceShapeAsync(plan, options, progress, cancellation.Token);

            if (result.Succeeded)
            {
                LastRunStatus = "Session trace complete";
                LastRunDetail = $"{plan.Shape}: trace completed through the persistent armed session. Amps remain enabled until Shutdown Session.";
                RobotSessionStatus = "Armed session ready - amps enabled";
                ModeState = "Session trace complete";
            }
            else
            {
                LastRunStatus = "Session trace failed";
                LastRunDetail = $"{plan.Shape}: session trace failed or was rejected. Check Process Log.";
                RobotSessionStatus = "Armed session ready - check trace result";
                ModeState = "Session trace failed";
            }
        }
        catch (OperationCanceledException)
        {
            LastRunStatus = "Session trace stopped";
            LastRunDetail = "Operator cancelled the active session trace wait.";
            AppendLog("ui", "Session trace wait stopped by operator. Use Stop Motion if the backend is still moving.");
            ModeState = "Stopped";
        }
        catch (Exception exception)
        {
            LastRunStatus = "Session trace error";
            LastRunDetail = exception.Message;
            RobotSessionStatus = "Session trace error";
            AppendLog("err", exception.Message);
            ModeState = "Error";
        }
        finally
        {
            _runCancellation = null;
            IsProcessActive = false;
            RefreshModeState();
            StartArmedSessionCommand.NotifyCanExecuteChanged();
            ShutdownSessionCommand.NotifyCanExecuteChanged();
            StopSessionMotionCommand.NotifyCanExecuteChanged();
            RunSessionShapeCommand.NotifyCanExecuteChanged();
        }
    }

    private async Task StartArmedSessionAsync()
    {
        RobotSessionStatus = "Starting armed session...";
        AppendLog("ui", "Start Armed Session requested. This phase runs rsiconfig once, starts the persistent C++ session, connects RMP, enables amps once, and can run validated session traces and backend-owned Cartesian jog commands.");

        try
        {
            var progress = new Progress<ProcessOutputLine>(line => AppendLog(line.Stream, line.Text));
            await _robotSessionService.StartAsync(progress, CancellationToken.None);
            RobotSessionStatus = "Armed session ready - amps enabled";
            AppendLog("ui", "Persistent armed session is ready. Amps remain enabled until Shutdown Session. Session trace commands are available; backend-owned Cartesian Z- jog is available through UI hold controls when Jog Mode is enabled.");
        }
        catch (Exception exception)
        {
            RobotSessionStatus = "Session start failed";
            AppendLog("err", $"Persistent session start failed: {exception.Message}");
        }
        finally
        {
            StartArmedSessionCommand.NotifyCanExecuteChanged();
            ShutdownSessionCommand.NotifyCanExecuteChanged();
            StopSessionMotionCommand.NotifyCanExecuteChanged();
            RunSessionShapeCommand.NotifyCanExecuteChanged();
            OnPropertyChanged(nameof(CanUseBackendJogControls));
        }
    }

    private async Task ShutdownSessionAsync()
    {
        RobotSessionStatus = "Shutting down session process...";
        AppendLog("ui", "Shutdown Session requested for persistent session process.");

        try
        {
            await _robotSessionService.ShutdownAsync(CancellationToken.None);
            RobotSessionStatus = "Not connected - armed session stopped.";
            IsBackendJogActive = false;
            ActiveJogDirection = string.Empty;
            AppendLog("ui", "Persistent session process shutdown complete.");
        }
        catch (Exception exception)
        {
            RobotSessionStatus = "Session shutdown failed";
            AppendLog("err", $"Persistent session shutdown failed: {exception.Message}");
        }
        finally
        {
            StartArmedSessionCommand.NotifyCanExecuteChanged();
            ShutdownSessionCommand.NotifyCanExecuteChanged();
            StopSessionMotionCommand.NotifyCanExecuteChanged();
            RunSessionShapeCommand.NotifyCanExecuteChanged();
            OnPropertyChanged(nameof(CanUseBackendJogControls));
        }
    }

    public async Task StartBackendCartesianJogAsync(string direction, string reason)
    {
        var normalizedDirection = NormalizeJogDirection(direction);
        if (normalizedDirection != "Z-")
        {
            AppendLog("ui", $"Backend Cartesian jog direction '{direction}' is not wired in the UI yet. Z- is the first validated direction.");
            return;
        }

        if (!CanUseBackendJogControls)
        {
            AppendLog("ui", "Backend Cartesian jog start rejected by UI. Start Armed Session first and enable Jog Mode. No motion command was sent.");
            return;
        }

        await _jogCommandSemaphore.WaitAsync();
        try
        {
            if (IsBackendJogActive)
            {
                if (string.Equals(ActiveJogDirection, normalizedDirection, StringComparison.OrdinalIgnoreCase))
                {
                    AppendLog("ui", $"Ignoring duplicate jog start for {normalizedDirection}; backend jog is already active.");
                    return;
                }

                AppendLog("ui", $"New jog direction {normalizedDirection} requested while {ActiveJogDirection} is active. Stopping active jog first.");
                await _robotSessionService.StopCartesianJogAsync("new jog direction requested", CancellationToken.None);
                IsBackendJogActive = false;
                ActiveJogDirection = string.Empty;
            }

            var speed = Math.Clamp(JogVelocity, 0.0001, 0.004);
            AppendLog("ui", $"UI backend jog start requested direction={normalizedDirection} speed={speed.ToString("0.######", CultureInfo.InvariantCulture)} reason={reason}. One start command only; backend owns endpoint-only PVT jog loop. v14 disables the unsafe rolling APPEND experiment and uses stable 500 ms non-append PVT smoothing spans.");
            RobotSessionStatus = $"Backend Cartesian jog {normalizedDirection} starting...";
            LastRunStatus = "Jog starting";
            LastRunDetail = $"Backend-owned endpoint-only Cartesian {normalizedDirection} jog requested at {speed.ToString("0.######", CultureInfo.InvariantCulture)} m/s. Default UI validation speed is 0.003 m/s; 0.004 m/s is the current guarded max.";

            await _robotSessionService.StartCartesianJogAsync(normalizedDirection, speed, CancellationToken.None);

            ActiveJogDirection = normalizedDirection;
            IsBackendJogActive = true;
            RobotSessionStatus = $"Backend Cartesian jog {normalizedDirection} active";
            LastRunStatus = "Jog running";
            LastRunDetail = $"Hold is active for backend-owned Cartesian {normalizedDirection}. Release, Space, Escape, or lost focus sends one jog stop.";
        }
        catch (Exception exception)
        {
            AppendLog("err", $"Backend Cartesian jog start failed: {exception.Message}");
            RobotSessionStatus = "Backend Cartesian jog start failed";
            LastRunStatus = "Jog start failed";
            LastRunDetail = exception.Message;
            IsBackendJogActive = false;
            ActiveJogDirection = string.Empty;
        }
        finally
        {
            _jogCommandSemaphore.Release();
        }
    }

    public async Task StopBackendCartesianJogAsync(string reason)
    {
        await _jogCommandSemaphore.WaitAsync();
        try
        {
            if (!IsBackendJogActive)
            {
                return;
            }

            var stoppedDirection = ActiveJogDirection;
            AppendLog("ui", $"UI backend jog stop requested reason={reason} activeDirection={stoppedDirection}. One stop command only; backend should decelerate/settle and keep amps enabled.");
            RobotSessionStatus = $"Stopping backend Cartesian jog {stoppedDirection}...";
            LastRunStatus = "Jog stopping";

            await _robotSessionService.StopCartesianJogAsync(reason, CancellationToken.None);

            IsBackendJogActive = false;
            ActiveJogDirection = string.Empty;
            RobotSessionStatus = "Armed session ready - amps should remain enabled after jog";
            LastRunStatus = "Jog stopped";
            LastRunDetail = $"Backend-owned Cartesian jog stopped by {reason}. Requesting status so logs show amp state.";

            await _robotSessionService.RequestStatusAsync(CancellationToken.None);
        }
        catch (Exception exception)
        {
            AppendLog("err", $"Backend Cartesian jog stop failed: {exception.Message}");
            RobotSessionStatus = "Backend Cartesian jog stop failed";
            LastRunStatus = "Jog stop failed";
            LastRunDetail = exception.Message;
        }
        finally
        {
            _jogCommandSemaphore.Release();
        }
    }

    private static string NormalizeJogDirection(string direction)
    {
        var normalized = direction.Trim().ToUpperInvariant();
        return normalized switch
        {
            "Z-" or "-Z" or "NEGATIVE_Z" or "ZNEGATIVE" => "Z-",
            "Z+" or "+Z" or "POSITIVE_Z" or "ZPOSITIVE" => "Z+",
            "X+" or "+X" or "POSITIVE_X" or "XPOSITIVE" => "X+",
            "X-" or "-X" or "NEGATIVE_X" or "XNEGATIVE" => "X-",
            "Y+" or "+Y" or "POSITIVE_Y" or "YPOSITIVE" => "Y+",
            "Y-" or "-Y" or "NEGATIVE_Y" or "YNEGATIVE" => "Y-",
            _ => normalized
        };
    }

    private async Task StopSessionMotionAsync()
    {
        RobotSessionStatus = "Stop requested for armed session.";
        AppendLog("ui", "Stop Motion requested for persistent armed session. Backend should abort active motion but keep amps enabled.");

        try
        {
            await _robotSessionService.StopMotionAsync(CancellationToken.None);
        }
        catch (Exception exception)
        {
            RobotSessionStatus = "Session stop failed";
            AppendLog("err", $"Persistent session stop failed: {exception.Message}");
        }
        finally
        {
            StartArmedSessionCommand.NotifyCanExecuteChanged();
            ShutdownSessionCommand.NotifyCanExecuteChanged();
            StopSessionMotionCommand.NotifyCanExecuteChanged();
            RunSessionShapeCommand.NotifyCanExecuteChanged();
            OnPropertyChanged(nameof(CanUseBackendJogControls));
        }
    }

    private IMotionChatService SelectMotionChatService()
    {
        if (!UseLlmAssistant)
        {
            MotionAssistantStatus = _localMotionChatService.StatusText;
            return _localMotionChatService;
        }

        MotionAssistantStatus = _llmMotionChatService.StatusText;
        return _llmMotionChatService.IsAvailable
            ? _llmMotionChatService
            : _localMotionChatService;
    }

    private void ApplyMotionChatResponse(MotionChatResponse response)
    {
        if (response.CanExecute)
        {
            AppendAssistantLog("Safety: planner execution permission was ignored and forced off.");
        }

        switch (response.Action)
        {
            case MotionChatAction.UpdatePlan:
                if (response.Plan == null)
                {
                    AppendAssistantLog("The planner did not return a complete plan. The current plan was not changed.");
                    return;
                }

                var clampedPlan = ClampPlan(response.Plan, out var clampMessages);
                ApplyMotionPlan(clampedPlan, disarmConfirmMotion: true);
                AppendAssistantLog(response.AssistantMessage);
                foreach (var clampMessage in clampMessages)
                {
                    AppendAssistantLog(clampMessage);
                }

                return;

            case MotionChatAction.ResetPlan:
                ApplyMotionPlan(CreateDefaultMotionPlan(), disarmConfirmMotion: true);
                IsDryRun = true;
                ConfirmMotion = false;
                AppendAssistantLog(response.AssistantMessage);
                return;

            case MotionChatAction.Explain:
            case MotionChatAction.Reject:
                AppendAssistantLog(response.AssistantMessage);
                return;

            default:
                AppendAssistantLog("The planner returned an unsupported action. The current plan was not changed.");
                return;
        }
    }

    private void ApplyMotionPlan(MotionPlan plan, bool disarmConfirmMotion)
    {
        SelectedShape = plan.Shape;
        CenterX = RoundMotionValue(plan.CenterX);
        CenterY = RoundMotionValue(plan.CenterY);
        CenterZ = RoundMotionValue(plan.CenterZ);
        ShapeSizeMeters = RoundMotionValue(plan.SizeMeters);
        Velocity = RoundMotionValue(plan.Velocity);

        if (disarmConfirmMotion && ConfirmMotion)
        {
            ConfirmMotion = false;
            AppendAssistantLog("Safety: chat plan changes disarmed Confirm Motion.");
        }
    }

    private MotionPlan ClampPlan(MotionPlan plan, out IReadOnlyList<string> clampMessages)
    {
        var messages = new List<string>();
        var clampedPlan = new MotionPlan
        {
            Shape = plan.Shape,
            CenterX = ClampPlanValue(plan.CenterX, MinCenterX, MaxCenterX, "Center X", messages),
            CenterY = ClampPlanValue(plan.CenterY, MinCenterY, MaxCenterY, "Center Y", messages),
            CenterZ = ClampPlanValue(plan.CenterZ, MinCenterZ, MaxCenterZ, "Center Z", messages),
            SizeMeters = ClampPlanValue(plan.SizeMeters, MinShapeSizeMeters, MaxShapeSizeMeters, "Size/radius", messages),
            Velocity = ClampPlanValue(plan.Velocity, MinVelocity, MaxVelocity, "Velocity", messages),
            CornerMode = string.IsNullOrWhiteSpace(plan.CornerMode) ? "sharp" : plan.CornerMode
        };

        clampMessages = messages;
        return clampedPlan;

        double ClampPlanValue(
            double value,
            double min,
            double max,
            string label,
            List<string> outputMessages)
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
            {
                outputMessages.Add($"{label} was invalid and was clamped to {min:0.0000}.");
                return min;
            }

            var clamped = Math.Clamp(value, min, max);
            if (Math.Abs(clamped - value) > 0.0000001)
            {
                outputMessages.Add(FormattableString.Invariant(
                    $"{label} was clamped from {value:0.0000} to {clamped:0.0000}."));
            }

            return clamped;
        }
    }

    private MotionPlan CreateCurrentMotionPlan()
    {
        return new MotionPlan
        {
            Shape = SelectedShape,
            CenterX = CenterX,
            CenterY = CenterY,
            CenterZ = CenterZ,
            SizeMeters = ShapeSizeMeters,
            Velocity = Velocity,
            CornerMode = "sharp"
        };
    }

    private MotionPlan CreateDefaultMotionPlan()
    {
        return new MotionPlan
        {
            Shape = DefaultShape,
            CenterX = _defaultCenter.X,
            CenterY = _defaultCenter.Y,
            CenterZ = _defaultCenter.Z,
            SizeMeters = _defaultShapeSizeMeters,
            Velocity = _defaultVelocity,
            CornerMode = "sharp"
        };
    }

    private static double RoundMotionValue(double value)
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
        RunSessionShapeCommand.NotifyCanExecuteChanged();
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

    private void RefreshMotionAssistantStatus()
    {
        MotionAssistantStatus = UseLlmAssistant
            ? _llmMotionChatService.StatusText
            : _localMotionChatService.StatusText;
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

