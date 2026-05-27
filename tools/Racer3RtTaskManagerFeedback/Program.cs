using Grpc.Core;
using Grpc.Net.Client;
using RSI.RapidCodeRemote;
using RSI.RapidServer;
using static RSI.RapidServer.ServerControlService;

static string GetArg(string[] args, string name, string defaultValue)
{
    for (int index = 0; index < args.Length - 1; index++)
    {
        if (string.Equals(args[index], name, StringComparison.OrdinalIgnoreCase))
        {
            return args[index + 1];
        }
    }
    return defaultValue;
}

static int GetIntArg(string[] args, string name, int defaultValue)
{
    return int.TryParse(GetArg(args, name, defaultValue.ToString()), out int value) ? value : defaultValue;
}

static double GetDoubleArg(string[] args, string name, double defaultValue)
{
    return double.TryParse(GetArg(args, name, defaultValue.ToString(System.Globalization.CultureInfo.InvariantCulture)), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out double value) ? value : defaultValue;
}

static bool HasFlag(string[] args, string name)
{
    return args.Any(arg => string.Equals(arg, name, StringComparison.OrdinalIgnoreCase));
}

static RequestHeader Header(bool info, bool status) => new()
{
    Optimization = new()
    {
        SkipConfig = true,
        SkipInfo = !info,
        SkipStatus = !status,
    }
};

static string FormatFirmwareValue(FirmwareValue value)
{
    return value.ValueCase switch
    {
        FirmwareValue.ValueOneofCase.BoolValue => value.BoolValue.ToString(),
        FirmwareValue.ValueOneofCase.Int8Value => value.Int8Value.ToString(),
        FirmwareValue.ValueOneofCase.Uint8Value => value.Uint8Value.ToString(),
        FirmwareValue.ValueOneofCase.Int16Value => value.Int16Value.ToString(),
        FirmwareValue.ValueOneofCase.Uint16Value => value.Uint16Value.ToString(),
        FirmwareValue.ValueOneofCase.Int32Value => value.Int32Value.ToString(),
        FirmwareValue.ValueOneofCase.Uint32Value => value.Uint32Value.ToString(),
        FirmwareValue.ValueOneofCase.Int64Value => value.Int64Value.ToString(),
        FirmwareValue.ValueOneofCase.Uint64Value => value.Uint64Value.ToString(),
        FirmwareValue.ValueOneofCase.FloatValue => value.FloatValue.ToString("G9"),
        FirmwareValue.ValueOneofCase.DoubleValue => value.DoubleValue.ToString("G17"),
        FirmwareValue.ValueOneofCase.None => "<none>",
        _ => value.ToString(),
    };
}

static long FirmwareValueAsInt64OrZero(FirmwareValue? value)
{
    if (value is null)
    {
        return 0;
    }

    return value.ValueCase switch
    {
        FirmwareValue.ValueOneofCase.Int8Value => value.Int8Value,
        FirmwareValue.ValueOneofCase.Uint8Value => value.Uint8Value,
        FirmwareValue.ValueOneofCase.Int16Value => value.Int16Value,
        FirmwareValue.ValueOneofCase.Uint16Value => value.Uint16Value,
        FirmwareValue.ValueOneofCase.Int32Value => value.Int32Value,
        FirmwareValue.ValueOneofCase.Uint32Value => value.Uint32Value,
        FirmwareValue.ValueOneofCase.Int64Value => value.Int64Value,
        FirmwareValue.ValueOneofCase.Uint64Value => value.Uint64Value > long.MaxValue ? long.MaxValue : (long)value.Uint64Value,
        FirmwareValue.ValueOneofCase.BoolValue => value.BoolValue ? 1 : 0,
        _ => 0,
    };
}

static double FirmwareValueAsDoubleOrZero(FirmwareValue? value)
{
    if (value is null)
    {
        return 0.0;
    }

    return value.ValueCase switch
    {
        FirmwareValue.ValueOneofCase.FloatValue => value.FloatValue,
        FirmwareValue.ValueOneofCase.DoubleValue => value.DoubleValue,
        FirmwareValue.ValueOneofCase.Int8Value => value.Int8Value,
        FirmwareValue.ValueOneofCase.Uint8Value => value.Uint8Value,
        FirmwareValue.ValueOneofCase.Int16Value => value.Int16Value,
        FirmwareValue.ValueOneofCase.Uint16Value => value.Uint16Value,
        FirmwareValue.ValueOneofCase.Int32Value => value.Int32Value,
        FirmwareValue.ValueOneofCase.Uint32Value => value.Uint32Value,
        FirmwareValue.ValueOneofCase.Int64Value => value.Int64Value,
        FirmwareValue.ValueOneofCase.Uint64Value => value.Uint64Value,
        _ => 0.0,
    };
}

static bool FirmwareValueAsBoolOrFalse(FirmwareValue? value)
{
    if (value is null)
    {
        return false;
    }

    return value.ValueCase switch
    {
        FirmwareValue.ValueOneofCase.BoolValue => value.BoolValue,
        _ => FirmwareValueAsInt64OrZero(value) != 0,
    };
}

static FirmwareValue? GetGlobal(RTTaskManagerStatus status, string name)
{
    return status.GlobalValues.TryGetValue(name, out FirmwareValue? value) ? value : null;
}

static RTTaskManagerAction.Types.GlobalValueSet GlobalValueSet(string name, FirmwareValue value, string libraryName, string libraryDirectory)
{
    RTTaskManagerAction.Types.GlobalValueSet set = new()
    {
        Name = name,
        Value = value,
    };

    if (!string.IsNullOrWhiteSpace(libraryName))
    {
        set.LibraryName = libraryName;
    }

    if (!string.IsNullOrWhiteSpace(libraryDirectory))
    {
        set.LibraryDirectory = libraryDirectory;
    }

    return set;
}

static async Task<RTTaskManagerStatus> ReadManagerStatusAsync(RMPService.RMPServiceClient rmp, int managerId)
{
    RTTaskManagerResponse response = await rmp.RTTaskManagerAsync(new RTTaskManagerRequest
    {
        Id = managerId,
        Header = Header(info: false, status: true),
    }, deadline: DateTime.UtcNow.AddSeconds(5));

    return response.Status;
}

static async Task<int> SelectManagerAsync(RMPService.RMPServiceClient rmp, IReadOnlyList<int> managerIds, string managerLabel)
{
    List<(int Id, string Label, RTTaskManagerResponse Response)> managers = [];

    foreach (int id in managerIds)
    {
        RTTaskManagerResponse response = await rmp.RTTaskManagerAsync(new RTTaskManagerRequest
        {
            Id = id,
            Header = Header(info: true, status: false),
        }, deadline: DateTime.UtcNow.AddSeconds(5));

        string label = response.Info?.CreationParameters?.UserLabel ?? "";
        string directory = response.Info?.CreationParameters?.RtTaskDirectory ?? "";
        string platform = response.Info?.CreationParameters?.Platform.ToString() ?? "";
        string nodeName = response.Info?.CreationParameters?.NodeName ?? "";

        Console.WriteLine($"  Manager id={id} label={label} platform={platform} node={nodeName} dir={directory}");
        managers.Add((id, label, response));
    }

    if (!string.IsNullOrWhiteSpace(managerLabel))
    {
        List<(int Id, string Label, RTTaskManagerResponse Response)> matches = managers
            .Where(manager => string.Equals(manager.Label, managerLabel, StringComparison.OrdinalIgnoreCase))
            .ToList();

        if (matches.Count == 1)
        {
            return matches[0].Id;
        }

        if (matches.Count > 1)
        {
            Console.WriteLine($"FAIL: multiple RTTaskManagers matched label '{managerLabel}'. Stop stale managers and retest.");
            Environment.Exit(2);
        }

        Console.WriteLine($"FAIL: no RTTaskManager with UserLabel '{managerLabel}' was discovered.");
        Environment.Exit(2);
    }

    return managers[0].Id;
}

string address = GetArg(args, "--address", "http://127.0.0.1:51061");
string managerLabel = GetArg(args, "--manager-label", "Racer3JogProbe");
string requiredAdvancingGlobal = GetArg(args, "--required-advancing-global", "");
string libraryName = GetArg(args, "--library-name", "RTTaskFunctions");
string libraryDirectory = GetArg(args, "--library-directory", "");
int iterations = GetIntArg(args, "--iterations", 5);
int delayMs = GetIntArg(args, "--delay-ms", 500);
bool requireCustomRacer3Globals = HasFlag(args, "--require-custom-racer3-globals");
bool exerciseJogIntent = HasFlag(args, "--exercise-jog-intent");
bool requireStatusReadiness = HasFlag(args, "--require-status-readiness");
bool allowMotion = HasFlag(args, "--allow-motion");
int jogAxisOneBased = GetIntArg(args, "--jog-axis", 6);
double jogStepUserUnits = GetDoubleArg(args, "--jog-step-user-units", 0.0002);
double jogSpeedUserUnitsPerSecond = GetDoubleArg(args, "--jog-speed-user-units-per-second", 0.002);
const double MaxAllowedJogStepUserUnits = 0.001;
const double MaxAllowedJogSpeedUserUnitsPerSecond = 0.005;

Console.WriteLine("Racer3 RTTaskManager feedback probe");
Console.WriteLine($"  RapidCodeRemote address: {address}");
Console.WriteLine($"  Target manager label:    {managerLabel}");
Console.WriteLine($"  Required advancing global: {(string.IsNullOrWhiteSpace(requiredAdvancingGlobal) ? "<any known heartbeat/counter>" : requiredAdvancingGlobal)}");
Console.WriteLine($"  Require Racer3 globals:  {requireCustomRacer3Globals}");
Console.WriteLine($"  Jog intent exercise:     {exerciseJogIntent}");
Console.WriteLine($"  Status readiness proof:  {requireStatusReadiness}");
Console.WriteLine($"  Allow motion writes:      {allowMotion}");
if (allowMotion)
{
    Console.WriteLine("  WARNING: --allow-motion arms one tiny RTTask endpoint-style Axis::MoveRelative command if all RTTask safety gates pass.");
    Console.WriteLine($"  Armed jog request:       axis={jogAxisOneBased} step={jogStepUserUnits:G17} user-units speed={jogSpeedUserUnitsPerSecond:G17} user-units/sec");
}
else
{
    Console.WriteLine("  This is readback/dry-run only. It does not command motion.");
}

using GrpcChannel channel = GrpcChannel.ForAddress(address);
ServerControlServiceClient server = new(channel);
RMPService.RMPServiceClient rmp = new(channel);

await server.GetInfoAsync(new(), deadline: DateTime.UtcNow.AddSeconds(5));
Console.WriteLine("rapidserver responded.");

RTTaskManagerResponse discoverResponse = await rmp.RTTaskManagerAsync(new RTTaskManagerRequest
{
    Header = Header(info: true, status: false),
    Action = new RTTaskManagerAction { Discover = new() },
}, deadline: DateTime.UtcNow.AddSeconds(10));

List<int> managerIds = discoverResponse.Action.Discover.ManagerIds.ToList();
Console.WriteLine($"Discovered manager IDs: {string.Join(", ", managerIds)}");
if (managerIds.Count == 0)
{
    Console.WriteLine("FAIL: no RTTaskManagers were discovered through rapidserver.");
    Environment.Exit(2);
}

int managerId = await SelectManagerAsync(rmp, managerIds, managerLabel);
Console.WriteLine($"Using manager id {managerId}.");

string[] requiredCustomGlobals =
[
    "initialized",
    "metadataReady",
    "runtimeReady",
    "statusSamplerReady",
    "basicHeartbeat",
    "heartbeat",
    "jogEnabled",
    "jogStopRequested",
    "jogMotionArmed",
    "jogCommandSequence",
    "jogDirectionCode",
    "jogTargetAxis",
    "jogSpeedMetersPerSecond",
    "jogSpeedUserUnitsPerSecond",
    "jogStepUserUnits",
    "lastJogCommandSequenceSeen",
    "jogIntentTransitions",
    "lastJogStopSeen",
    "lastMotionCommandSequenceAccepted",
    "lastMotionCommandSequenceRejected",
    "lastMotionRejectCode",
    "lastMotionCommandIssued",
    "lastMotionCommandDone",
    "lastMotionAxis",
    "lastMotionStepUserUnits",
    "lastMotionSpeedUserUnitsPerSecond",
    "initializationCount",
    "lastSampleCounter",
    "lastNetworkCounter",
    "statusSampleCount",
    "samplePeriodSeconds",
    "lastInitializationStep",
    "lastStatusSamplerStep",
    "lastStatusReadinessCode",
    "multiAxisReady",
    "multiAxisAmpEnabled",
    "allAxisAmpEnabled",
    "targetAxisAmpEnabled",
    "taskErrorCount",
    "lastErrorCode",
];

if (exerciseJogIntent)
{
    requireCustomRacer3Globals = true;
}

if (requireStatusReadiness)
{
    requireCustomRacer3Globals = true;
}

if (allowMotion)
{
    exerciseJogIntent = true;
    requireStatusReadiness = true;
    requireCustomRacer3Globals = true;

    if (jogAxisOneBased < 1 || jogAxisOneBased > 6)
    {
        Console.WriteLine("FAIL: --jog-axis must be in the operator-facing range 1..6.");
        Environment.Exit(6);
    }

    if (jogStepUserUnits == 0.0 || Math.Abs(jogStepUserUnits) > MaxAllowedJogStepUserUnits)
    {
        Console.WriteLine($"FAIL: --jog-step-user-units must be nonzero and <= {MaxAllowedJogStepUserUnits:G17} in magnitude.");
        Environment.Exit(6);
    }

    if (jogSpeedUserUnitsPerSecond <= 0.0 || jogSpeedUserUnitsPerSecond > MaxAllowedJogSpeedUserUnitsPerSecond)
    {
        Console.WriteLine($"FAIL: --jog-speed-user-units-per-second must be > 0 and <= {MaxAllowedJogSpeedUserUnitsPerSecond:G17}.");
        Environment.Exit(6);
    }
}

long jogSequenceWritten = -1;
long jogTransitionsBeforeWrite = -1;
if (exerciseJogIntent)
{
    RTTaskManagerStatus beforeWrite = await ReadManagerStatusAsync(rmp, managerId);
    string[] requiredJogGlobals = ["jogCommandSequence", "lastJogCommandSequenceSeen", "jogIntentTransitions"];
    string[] missingJogGlobals = requiredJogGlobals
        .Where(name => GetGlobal(beforeWrite, name) is null)
        .ToArray();

    if (missingJogGlobals.Length > 0)
    {
        Console.WriteLine($"FAIL: cannot run jog-intent readback because these globals are missing: {string.Join(", ", missingJogGlobals)}");
        Environment.Exit(4);
    }

    long currentSequence = FirmwareValueAsInt64OrZero(GetGlobal(beforeWrite, "jogCommandSequence"));
    long lastSequenceSeen = FirmwareValueAsInt64OrZero(GetGlobal(beforeWrite, "lastJogCommandSequenceSeen"));
    jogTransitionsBeforeWrite = FirmwareValueAsInt64OrZero(GetGlobal(beforeWrite, "jogIntentTransitions"));
    jogSequenceWritten = Math.Max(currentSequence, lastSequenceSeen) + 1;

    int jogAxisZeroBased = jogAxisOneBased - 1;
    double signedStep = Math.CopySign(Math.Abs(jogStepUserUnits), jogStepUserUnits == 0.0 ? 1.0 : jogStepUserUnits);
    int directionCode = signedStep >= 0.0 ? 1 : -1;

    RTTaskManagerAction writeAction = new();
    writeAction.GlobalValueSets.Add(GlobalValueSet("jogEnabled", new FirmwareValue { BoolValue = allowMotion }, libraryName, libraryDirectory));
    writeAction.GlobalValueSets.Add(GlobalValueSet("jogStopRequested", new FirmwareValue { BoolValue = false }, libraryName, libraryDirectory));
    writeAction.GlobalValueSets.Add(GlobalValueSet("jogMotionArmed", new FirmwareValue { BoolValue = allowMotion }, libraryName, libraryDirectory));
    writeAction.GlobalValueSets.Add(GlobalValueSet("jogTargetAxis", new FirmwareValue { Int32Value = jogAxisZeroBased }, libraryName, libraryDirectory));
    writeAction.GlobalValueSets.Add(GlobalValueSet("jogDirectionCode", new FirmwareValue { Int32Value = directionCode }, libraryName, libraryDirectory));
    writeAction.GlobalValueSets.Add(GlobalValueSet("jogSpeedMetersPerSecond", new FirmwareValue { DoubleValue = 0.0 }, libraryName, libraryDirectory));
    writeAction.GlobalValueSets.Add(GlobalValueSet("jogSpeedUserUnitsPerSecond", new FirmwareValue { DoubleValue = allowMotion ? jogSpeedUserUnitsPerSecond : 0.0 }, libraryName, libraryDirectory));
    writeAction.GlobalValueSets.Add(GlobalValueSet("jogStepUserUnits", new FirmwareValue { DoubleValue = allowMotion ? signedStep : 0.0 }, libraryName, libraryDirectory));
    writeAction.GlobalValueSets.Add(GlobalValueSet("jogCommandSequence", new FirmwareValue { Int64Value = jogSequenceWritten }, libraryName, libraryDirectory));

    await rmp.RTTaskManagerAsync(new RTTaskManagerRequest
    {
        Id = managerId,
        Header = Header(info: false, status: false),
        Action = writeAction,
    }, deadline: DateTime.UtcNow.AddSeconds(5));

    if (allowMotion)
    {
        Console.WriteLine($"Wrote ARMED tiny jog intent: jogEnabled=true jogMotionArmed=true axis={jogAxisOneBased} step={signedStep:G17} user-units speed={jogSpeedUserUnitsPerSecond:G17} user-units/sec jogCommandSequence={jogSequenceWritten}");
    }
    else
    {
        Console.WriteLine($"Wrote dry-run jog intent: jogEnabled=false jogMotionArmed=false jogStopRequested=false jogCommandSequence={jogSequenceWritten}");
    }
}

long firstCounter = -1;
long lastCounter = -1;
long firstHeartbeat = -1;
long lastHeartbeat = -1;
long firstBasicHeartbeat = -1;
long lastBasicHeartbeat = -1;
long? firstRequiredGlobal = null;
long? lastRequiredGlobal = null;
bool requiredGlobalSeen = false;
HashSet<string> observedGlobals = new(StringComparer.OrdinalIgnoreCase);
RTTaskManagerStatus? lastStatus = null;
long firstStatusSampleCount = -1;
long lastStatusSampleCount = -1;
long firstLastSampleCounter = -1;
long lastLastSampleCounter = -1;
long firstLastNetworkCounter = -1;
long lastLastNetworkCounter = -1;
long firstStatusSamplerExecutionCount = -1;
long lastStatusSamplerExecutionCount = -1;
string lastInitializeTaskState = "<missing>";
string lastStatusSamplerTaskState = "<missing>";
string lastInitializeTaskError = "";
string lastStatusSamplerTaskError = "";

string[] preferredGlobalsToPrint =
[
    "initialized",
    "metadataReady",
    "runtimeReady",
    "statusSamplerReady",
    "counter",
    "basicHeartbeat",
    "heartbeat",
    "initializationCount",
    "startingSample",
    "sensorValue",
    "lastSampleCounter",
    "lastNetworkCounter",
    "statusSampleCount",
    "samplePeriodSeconds",
    "lastInitializationStep",
    "lastStatusSamplerStep",
    "lastStatusReadinessCode",
    "jogEnabled",
    "jogStopRequested",
    "jogMotionArmed",
    "jogCommandSequence",
    "jogDirectionCode",
    "jogTargetAxis",
    "jogSpeedMetersPerSecond",
    "jogSpeedUserUnitsPerSecond",
    "jogStepUserUnits",
    "lastJogCommandSequenceSeen",
    "jogIntentTransitions",
    "lastJogStopSeen",
    "lastMotionCommandSequenceAccepted",
    "lastMotionCommandSequenceRejected",
    "lastMotionRejectCode",
    "lastMotionCommandIssued",
    "lastMotionCommandDone",
    "lastMotionAxis",
    "lastMotionStepUserUnits",
    "lastMotionSpeedUserUnitsPerSecond",
    "multiAxisReady",
    "multiAxisAmpEnabled",
    "allAxisAmpEnabled",
    "targetAxisAmpEnabled",
    "taskErrorCount",
    "lastErrorCode",
];

for (int iteration = 1; iteration <= iterations; iteration++)
{
    RTTaskManagerStatus status = await ReadManagerStatusAsync(rmp, managerId);
    lastStatus = status;
    foreach (string name in status.GlobalValues.Keys)
    {
        observedGlobals.Add(name);
    }

    Console.WriteLine();
    Console.WriteLine($"Iteration {iteration}/{iterations}");
    Console.WriteLine($"  Manager state: {status.State}");
    Console.WriteLine($"  CycleCount:    {status.CycleCount}");
    Console.WriteLine($"  TaskIds:       {string.Join(", ", status.TaskIds)}");

    Console.WriteLine("  Globals:");
    HashSet<string> printedGlobals = new(StringComparer.OrdinalIgnoreCase);
    foreach (string name in preferredGlobalsToPrint)
    {
        FirmwareValue? value = GetGlobal(status, name);
        Console.WriteLine($"    {name} = {(value is null ? "<missing>" : FormatFirmwareValue(value))}");
        printedGlobals.Add(name);
    }

    foreach (string name in status.GlobalValues.Keys.OrderBy(name => name, StringComparer.OrdinalIgnoreCase))
    {
        if (printedGlobals.Contains(name))
        {
            continue;
        }

        FirmwareValue? value = GetGlobal(status, name);
        Console.WriteLine($"    {name} = {(value is null ? "<missing>" : FormatFirmwareValue(value))}");
    }

    long counter = FirmwareValueAsInt64OrZero(GetGlobal(status, "counter"));
    long heartbeat = FirmwareValueAsInt64OrZero(GetGlobal(status, "heartbeat"));
    long basicHeartbeat = FirmwareValueAsInt64OrZero(GetGlobal(status, "basicHeartbeat"));
    long statusSampleCount = FirmwareValueAsInt64OrZero(GetGlobal(status, "statusSampleCount"));
    long lastSampleCounterValue = FirmwareValueAsInt64OrZero(GetGlobal(status, "lastSampleCounter"));
    long lastNetworkCounterValue = FirmwareValueAsInt64OrZero(GetGlobal(status, "lastNetworkCounter"));
    if (firstCounter < 0)
    {
        firstCounter = counter;
        firstHeartbeat = heartbeat;
        firstBasicHeartbeat = basicHeartbeat;
        firstStatusSampleCount = statusSampleCount;
        firstLastSampleCounter = lastSampleCounterValue;
        firstLastNetworkCounter = lastNetworkCounterValue;
    }
    lastCounter = counter;
    lastHeartbeat = heartbeat;
    lastBasicHeartbeat = basicHeartbeat;
    lastStatusSampleCount = statusSampleCount;
    lastLastSampleCounter = lastSampleCounterValue;
    lastLastNetworkCounter = lastNetworkCounterValue;

    if (!string.IsNullOrWhiteSpace(requiredAdvancingGlobal))
    {
        FirmwareValue? requiredValue = GetGlobal(status, requiredAdvancingGlobal);
        if (requiredValue is not null)
        {
            long numericRequiredValue = FirmwareValueAsInt64OrZero(requiredValue);
            requiredGlobalSeen = true;
            firstRequiredGlobal ??= numericRequiredValue;
            lastRequiredGlobal = numericRequiredValue;
        }
    }

    if (status.TaskIds.Count > 0)
    {
        RTTaskBatchRequest batchRequest = new();
        foreach (int taskId in status.TaskIds)
        {
            batchRequest.Requests.Add(new RTTaskRequest
            {
                Id = taskId,
                ManagerId = managerId,
                Header = Header(info: true, status: true),
            });
        }

        RTTaskBatchResponse taskBatch = await rmp.RTTaskBatchAsync(batchRequest, deadline: DateTime.UtcNow.AddSeconds(5));
        Console.WriteLine("  Tasks:");
        foreach (RTTaskResponse taskResponse in taskBatch.Responses)
        {
            string functionName = taskResponse.Info?.CreationParameters?.FunctionName ?? "<unknown>";
            string userLabel = taskResponse.Info?.CreationParameters?.UserLabel ?? "";
            string taskState = taskResponse.Status?.State.ToString() ?? "<unknown>";
            string executionCount = taskResponse.Status?.ExecutionCount.ToString() ?? "<unknown>";
            string period = taskResponse.Info?.CreationParameters?.Period.ToString() ?? "<unknown>";
            string errorMessage = taskResponse.Status?.ErrorMessage ?? "";
            string errorSuffix = string.IsNullOrWhiteSpace(errorMessage) ? "" : $" error=\"{errorMessage}\"";
            Console.WriteLine($"    id={taskResponse.Id} function={functionName} label={userLabel} state={taskState} executions={executionCount} period={period}{errorSuffix}");

            long numericExecutionCount = long.TryParse(executionCount, out long parsedExecutionCount) ? parsedExecutionCount : -1;
            if (string.Equals(functionName, "Initialize", StringComparison.OrdinalIgnoreCase))
            {
                lastInitializeTaskState = taskState;
                lastInitializeTaskError = errorMessage;
            }
            else if (string.Equals(functionName, "Racer3StatusSampler", StringComparison.OrdinalIgnoreCase))
            {
                if (firstStatusSamplerExecutionCount < 0)
                {
                    firstStatusSamplerExecutionCount = numericExecutionCount;
                }
                lastStatusSamplerExecutionCount = numericExecutionCount;
                lastStatusSamplerTaskState = taskState;
                lastStatusSamplerTaskError = errorMessage;
            }
        }
    }

    if (iteration < iterations)
    {
        await Task.Delay(delayMs);
    }
}

Console.WriteLine();
Console.WriteLine($"Counter delta:        {lastCounter - firstCounter}");
Console.WriteLine($"Heartbeat delta:      {lastHeartbeat - firstHeartbeat}");
Console.WriteLine($"Basic heartbeat delta:{lastBasicHeartbeat - firstBasicHeartbeat}");
Console.WriteLine($"Status sample delta:  {lastStatusSampleCount - firstStatusSampleCount}");
Console.WriteLine($"LastSampleCounter delta: {lastLastSampleCounter - firstLastSampleCounter}");
Console.WriteLine($"LastNetworkCounter delta:{lastLastNetworkCounter - firstLastNetworkCounter}");

if (requireCustomRacer3Globals)
{
    string[] missingCustomGlobals = requiredCustomGlobals
        .Where(name => !observedGlobals.Contains(name))
        .ToArray();

    if (missingCustomGlobals.Length > 0)
    {
        Console.WriteLine($"FAIL: Racer3 custom globals were missing from RTTaskManager feedback: {string.Join(", ", missingCustomGlobals)}");
        Console.WriteLine("This usually means RapidCodeRemote loaded stale host metadata from RTTaskFunctions.dll instead of the Racer3 metadata companion.");
        Environment.Exit(3);
    }

    Console.WriteLine("Racer3 custom global metadata is present.");
}

if (exerciseJogIntent)
{
    if (lastStatus is null)
    {
        Console.WriteLine("FAIL: no manager status was read after jog-intent write.");
        Environment.Exit(4);
    }

    long finalCommandSequence = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "jogCommandSequence"));
    long finalSequenceSeen = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "lastJogCommandSequenceSeen"));
    long finalTransitions = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "jogIntentTransitions"));
    long finalAccepted = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "lastMotionCommandSequenceAccepted"));
    long finalRejected = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "lastMotionCommandSequenceRejected"));
    long finalRejectCode = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "lastMotionRejectCode"));
    bool finalIssued = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "lastMotionCommandIssued"));
    bool finalDone = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "lastMotionCommandDone"));

    Console.WriteLine($"Jog intent sequence written: {jogSequenceWritten}");
    Console.WriteLine($"Jog intent sequence read:    {finalCommandSequence}");
    Console.WriteLine($"Last sequence seen by task:  {finalSequenceSeen}");
    Console.WriteLine($"Jog transition delta:        {finalTransitions - jogTransitionsBeforeWrite}");
    Console.WriteLine($"Motion accepted sequence:    {finalAccepted}");
    Console.WriteLine($"Motion rejected sequence:    {finalRejected}");
    Console.WriteLine($"Motion reject code:          {finalRejectCode}");
    Console.WriteLine($"Motion issued/done:          {finalIssued}/{finalDone}");

    if (finalCommandSequence != jogSequenceWritten)
    {
        Console.WriteLine("FAIL: jogCommandSequence did not retain the written no-motion intent value.");
        Environment.Exit(4);
    }

    if (finalSequenceSeen < jogSequenceWritten || finalTransitions <= jogTransitionsBeforeWrite)
    {
        Console.WriteLine("FAIL: Racer3JogIntentMonitor did not observe the no-motion jog intent command sequence.");
        Environment.Exit(4);
    }

    if (!allowMotion)
    {
        if (finalIssued || finalAccepted == jogSequenceWritten)
        {
            Console.WriteLine("FAIL: dry-run jog intent unexpectedly issued or accepted a motion command.");
            Environment.Exit(4);
        }
        Console.WriteLine("PASS: no-motion jog-intent globals were written and observed by Racer3JogIntentMonitor; motion remained disarmed.");
    }
    else
    {
        if (finalAccepted == jogSequenceWritten && finalIssued)
        {
            Console.WriteLine("PASS: ARMED tiny RTTask jog command was accepted/issued by Racer3JogIntentMonitor.");
        }
        else if (finalRejected == jogSequenceWritten && finalRejectCode != 0)
        {
            Console.WriteLine("PASS: ARMED tiny RTTask jog command was safely rejected by an RTTask motion gate.");
        }
        else
        {
            Console.WriteLine("FAIL: armed jog intent was observed but neither accepted nor explicitly rejected.");
            Environment.Exit(6);
        }

        RTTaskManagerAction clearAction = new();
        clearAction.GlobalValueSets.Add(GlobalValueSet("jogEnabled", new FirmwareValue { BoolValue = false }, libraryName, libraryDirectory));
        clearAction.GlobalValueSets.Add(GlobalValueSet("jogMotionArmed", new FirmwareValue { BoolValue = false }, libraryName, libraryDirectory));
        clearAction.GlobalValueSets.Add(GlobalValueSet("jogStopRequested", new FirmwareValue { BoolValue = true }, libraryName, libraryDirectory));
        await rmp.RTTaskManagerAsync(new RTTaskManagerRequest
        {
            Id = managerId,
            Header = Header(info: false, status: false),
            Action = clearAction,
        }, deadline: DateTime.UtcNow.AddSeconds(5));
        Console.WriteLine("Cleared armed jog globals after proof: jogEnabled=false jogMotionArmed=false jogStopRequested=true.");
    }
}

if (requireStatusReadiness)
{
    if (lastStatus is null)
    {
        Console.WriteLine("FAIL: no manager status was read for status-readiness proof.");
        Environment.Exit(5);
    }

    bool initialized = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "initialized"));
    bool metadataReady = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "metadataReady"));
    bool runtimeReady = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "runtimeReady"));
    bool statusSamplerReady = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "statusSamplerReady"));
    bool multiAxisReady = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "multiAxisReady"));
    bool multiAxisAmpEnabled = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "multiAxisAmpEnabled"));
    bool allAxisAmpEnabled = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "allAxisAmpEnabled"));
    bool targetAxisAmpEnabled = FirmwareValueAsBoolOrFalse(GetGlobal(lastStatus, "targetAxisAmpEnabled"));
    long initializationCount = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "initializationCount"));
    long taskErrorCount = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "taskErrorCount"));
    long lastErrorCode = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "lastErrorCode"));
    long lastInitializationStep = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "lastInitializationStep"));
    long lastStatusSamplerStep = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "lastStatusSamplerStep"));
    long lastStatusReadinessCode = FirmwareValueAsInt64OrZero(GetGlobal(lastStatus, "lastStatusReadinessCode"));
    double samplePeriodSeconds = FirmwareValueAsDoubleOrZero(GetGlobal(lastStatus, "samplePeriodSeconds"));
    long statusSamplerExecutionDelta = lastStatusSamplerExecutionCount - firstStatusSamplerExecutionCount;

    Console.WriteLine("Status readiness summary:");
    Console.WriteLine($"  initialized={initialized} metadataReady={metadataReady} runtimeReady={runtimeReady} statusSamplerReady={statusSamplerReady}");
    Console.WriteLine($"  initializationCount={initializationCount} taskErrorCount={taskErrorCount} lastErrorCode={lastErrorCode}");
    Console.WriteLine($"  lastInitializationStep={lastInitializationStep} lastStatusSamplerStep={lastStatusSamplerStep} lastStatusReadinessCode={lastStatusReadinessCode}");
    Console.WriteLine($"  statusSampleCount={lastStatusSampleCount} statusSampleDelta={lastStatusSampleCount - firstStatusSampleCount}");
    Console.WriteLine($"  lastSampleCounter={lastLastSampleCounter} delta={lastLastSampleCounter - firstLastSampleCounter}");
    Console.WriteLine($"  lastNetworkCounter={lastLastNetworkCounter} delta={lastLastNetworkCounter - firstLastNetworkCounter}");
    Console.WriteLine($"  samplePeriodSeconds={samplePeriodSeconds:G17}");
    Console.WriteLine($"  multiAxisReady={multiAxisReady} multiAxisAmpEnabled={multiAxisAmpEnabled} allAxisAmpEnabled={allAxisAmpEnabled} targetAxisAmpEnabled={targetAxisAmpEnabled}");
    Console.WriteLine($"  Initialize task state={lastInitializeTaskState}{(string.IsNullOrWhiteSpace(lastInitializeTaskError) ? "" : $" error=\"{lastInitializeTaskError}\"")}");
    Console.WriteLine($"  Racer3StatusSampler task state={lastStatusSamplerTaskState} executionDelta={statusSamplerExecutionDelta}{(string.IsNullOrWhiteSpace(lastStatusSamplerTaskError) ? "" : $" error=\"{lastStatusSamplerTaskError}\"")}");

    bool statusCountersAdvanced = (lastStatusSampleCount > firstStatusSampleCount) &&
        (lastLastSampleCounter > firstLastSampleCounter || lastLastNetworkCounter > firstLastNetworkCounter);
    bool samplePeriodSane = samplePeriodSeconds > 0.0 && samplePeriodSeconds < 1.0;
    bool samplerExecuting = firstStatusSamplerExecutionCount >= 0 && lastStatusSamplerExecutionCount > firstStatusSamplerExecutionCount;

    if (initialized && metadataReady && runtimeReady && statusSamplerReady && multiAxisReady &&
        initializationCount > 0 && statusCountersAdvanced && samplePeriodSane && samplerExecuting)
    {
        Console.WriteLine("PASS: initialization/status readiness is established without commanding motion.");
    }
    else
    {
        bool failureExplained = taskErrorCount > 0 && lastErrorCode != 0 &&
            (lastInitializationStep != 0 || lastStatusSamplerStep != 0 || lastStatusReadinessCode != 0);
        Console.WriteLine(failureExplained
            ? "FAIL: initialization/status readiness is not established; diagnostic globals identify the failing step."
            : "FAIL: initialization/status readiness is not established and no diagnostic failure code was reported.");
        Environment.Exit(5);
    }
}

if (!string.IsNullOrWhiteSpace(requiredAdvancingGlobal))
{
    if (!requiredGlobalSeen || firstRequiredGlobal is null || lastRequiredGlobal is null)
    {
        Console.WriteLine($"FAIL: required global '{requiredAdvancingGlobal}' was not present in RTTaskManager feedback.");
        Environment.Exit(3);
    }

    Console.WriteLine($"Required global '{requiredAdvancingGlobal}' delta: {lastRequiredGlobal.Value - firstRequiredGlobal.Value}");
    if (lastRequiredGlobal > firstRequiredGlobal)
    {
        Console.WriteLine($"PASS: RTTaskManager '{managerLabel}' was discovered and required global '{requiredAdvancingGlobal}' advanced.");
        Environment.Exit(0);
    }

    Console.WriteLine($"FAIL: required global '{requiredAdvancingGlobal}' was present but did not advance.");
    Environment.Exit(3);
}

if (lastCounter > firstCounter || lastHeartbeat > firstHeartbeat || lastBasicHeartbeat > firstBasicHeartbeat)
{
    Console.WriteLine("PASS: RTTaskManager was discovered and task/global feedback advanced.");
    Environment.Exit(0);
}

Console.WriteLine("FAIL: RTTaskManager was discovered but counter/heartbeat globals did not advance.");
Environment.Exit(3);
