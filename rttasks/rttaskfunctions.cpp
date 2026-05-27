#include "rttaskglobals.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <exception>

using namespace RSI::RapidCode;
using namespace RSI::RapidCode::RealTimeTasks;

namespace
{
constexpr int Racer3AxisCount = 6;
constexpr int Racer3RtTaskMultiAxisIndex = 0;
constexpr double Racer3RtTaskJogMaxStepUserUnits = 0.001;
constexpr double Racer3RtTaskJogMaxSpeedUserUnitsPerSecond = 0.005;
constexpr double Racer3RtTaskJogDefaultAcceleration = 0.05;
constexpr double Racer3RtTaskJogDefaultDeceleration = 0.05;
constexpr double Racer3RtTaskJogDefaultJerkPercent = 5.0;

enum Racer3RtTaskJogRejectCode : int32_t
{
    Racer3JogRejectNone = 0,
    Racer3JogRejectNotArmed = 1,
    Racer3JogRejectNotInitialized = 2,
    Racer3JogRejectStatusNotReady = 3,
    Racer3JogRejectMultiAxisNotReady = 4,
    Racer3JogRejectStopRequested = 5,
    Racer3JogRejectInvalidAxis = 6,
    Racer3JogRejectInvalidSpeed = 7,
    Racer3JogRejectInvalidStep = 8,
    Racer3JogRejectMotionApiError = 9,
    Racer3JogRejectAmpsDisabled = 10,
    Racer3JogRejectCommandAlreadySeen = 11,
    Racer3JogRejectJogDisabled = 12,
};

constexpr int32_t Racer3ReadinessStepNotStarted = 0;
constexpr int32_t Racer3ReadinessStepGlobalsReset = 10;
constexpr int32_t Racer3ReadinessStepControllerHandle = 20;
constexpr int32_t Racer3ReadinessStepControllerStatus = 30;
constexpr int32_t Racer3ReadinessStepMultiAxisHandle = 40;
constexpr int32_t Racer3ReadinessStepMultiAxisStatus = 50;
constexpr int32_t Racer3ReadinessStepAxisStatus = 60;
constexpr int32_t Racer3ReadinessStepComplete = 100;

void recordTaskError(GlobalData* data, int32_t errorCode)
{
    data->taskErrorCount += 1;
    data->lastErrorCode = errorCode;
}

int32_t readinessErrorCodeFromStep(int32_t step)
{
    if (step <= Racer3ReadinessStepNotStarted)
    {
        step = Racer3ReadinessStepNotStarted;
    }
    return -step;
}

void recordReadinessError(GlobalData* data, std::atomic<int32_t>& stepGlobal)
{
    const int32_t errorCode = readinessErrorCodeFromStep(stepGlobal.load());
    data->statusSamplerReady = false;
    data->initialized = false;
    data->lastStatusReadinessCode = errorCode;
    recordTaskError(data, errorCode);
}

void setAxisPosition(GlobalData* data, int index, double commandPosition, double actualPosition)
{
    switch (index)
    {
    case 0:
        data->axis0CommandPosition = commandPosition;
        data->axis0ActualPosition = actualPosition;
        break;
    case 1:
        data->axis1CommandPosition = commandPosition;
        data->axis1ActualPosition = actualPosition;
        break;
    case 2:
        data->axis2CommandPosition = commandPosition;
        data->axis2ActualPosition = actualPosition;
        break;
    case 3:
        data->axis3CommandPosition = commandPosition;
        data->axis3ActualPosition = actualPosition;
        break;
    case 4:
        data->axis4CommandPosition = commandPosition;
        data->axis4ActualPosition = actualPosition;
        break;
    case 5:
        data->axis5CommandPosition = commandPosition;
        data->axis5ActualPosition = actualPosition;
        break;
    default:
        break;
    }
}

void sampleRacer3MotionState(GlobalData* data, std::atomic<int32_t>& stepGlobal)
{
    data->statusSamplerReady = false;
    data->multiAxisReady = false;
    data->multiAxisAmpEnabled = false;
    data->multiAxisMotionDone = false;
    data->allAxisAmpEnabled = false;
    data->allAxisMotionDone = false;

    stepGlobal = Racer3ReadinessStepControllerHandle;
    auto* controller = Racer3RTMotionControllerGet();
    stepGlobal = Racer3ReadinessStepControllerStatus;
    const double sampleRate = static_cast<double>(controller->SampleRateGet());

    if (sampleRate > 0.0)
    {
        data->samplePeriodSeconds = 1.0 / sampleRate;
    }

    data->lastSampleCounter = controller->SampleCounterGet();
    data->lastNetworkCounter = controller->NetworkCounterGet();
    data->runtimeReady = true;

    stepGlobal = Racer3ReadinessStepMultiAxisHandle;
    auto* multiAxis = Racer3RTMultiAxisGet(Racer3RtTaskMultiAxisIndex);

    stepGlobal = Racer3ReadinessStepMultiAxisStatus;
    data->multiAxisReady = true;
    data->multiAxisAmpEnabled = multiAxis->AmpEnableGet();
    data->multiAxisMotionDone = multiAxis->MotionDoneGet();
    data->multiAxisState = static_cast<int32_t>(multiAxis->StateGet());
    data->multiAxisMotionId = static_cast<int32_t>(multiAxis->MotionIdGet());
    data->multiAxisExecutingMotionId = static_cast<int32_t>(multiAxis->MotionIdExecutingGet());

    bool allAmpEnabled = data->multiAxisAmpEnabled;
    bool allMotionDone = data->multiAxisMotionDone;
    stepGlobal = Racer3ReadinessStepAxisStatus;
    for (int index = 0; index < Racer3AxisCount; ++index)
    {
        auto* axis = Racer3RTAxisGet(index);
        allAmpEnabled = allAmpEnabled && axis->AmpEnableGet();
        allMotionDone = allMotionDone && axis->MotionDoneGet();
        setAxisPosition(data, index, axis->CommandPositionGet(), axis->ActualPositionGet());
    }

    data->allAxisAmpEnabled = allAmpEnabled;
    data->allAxisMotionDone = allMotionDone;
    data->statusSampleCount += 1;
    data->statusSamplerReady = true;
    data->initialized = true;
    data->lastStatusReadinessCode = Racer3ReadinessStepComplete;
    stepGlobal = Racer3ReadinessStepComplete;
}

void rejectJogMotion(GlobalData* data, int64_t sequence, int32_t rejectCode)
{
    data->lastMotionCommandSequenceRejected = sequence;
    data->lastMotionRejectCode = rejectCode;
    data->lastMotionCommandIssued = false;
}

void issueTinyJogStepIfArmed(GlobalData* data, int64_t sequence)
{
    data->lastMotionRejectCode = Racer3JogRejectNone;
    data->lastMotionCommandIssued = false;
    data->lastMotionCommandDone = false;

    if (!data->jogMotionArmed)
    {
        rejectJogMotion(data, sequence, Racer3JogRejectNotArmed);
        return;
    }

    if (!data->initialized)
    {
        rejectJogMotion(data, sequence, Racer3JogRejectNotInitialized);
        return;
    }

    if (!data->statusSamplerReady)
    {
        rejectJogMotion(data, sequence, Racer3JogRejectStatusNotReady);
        return;
    }

    if (!data->multiAxisReady)
    {
        rejectJogMotion(data, sequence, Racer3JogRejectMultiAxisNotReady);
        return;
    }

    if (data->jogStopRequested)
    {
        data->lastJogStopSeen = sequence;
        rejectJogMotion(data, sequence, Racer3JogRejectStopRequested);
        return;
    }

    if (!data->jogEnabled)
    {
        rejectJogMotion(data, sequence, Racer3JogRejectJogDisabled);
        return;
    }

    const int32_t axisIndex = data->jogTargetAxis.load();
    if (axisIndex < 0 || axisIndex >= Racer3AxisCount)
    {
        data->targetAxisAmpEnabled = false;
        rejectJogMotion(data, sequence, Racer3JogRejectInvalidAxis);
        return;
    }

    auto* axis = Racer3RTAxisGet(axisIndex);
    data->targetAxisAmpEnabled = axis->AmpEnableGet();
    if (!data->targetAxisAmpEnabled)
    {
        rejectJogMotion(data, sequence, Racer3JogRejectAmpsDisabled);
        return;
    }

    const double requestedStep = data->jogStepUserUnits.load();
    if (requestedStep == 0.0 || std::abs(requestedStep) > Racer3RtTaskJogMaxStepUserUnits)
    {
        rejectJogMotion(data, sequence, Racer3JogRejectInvalidStep);
        return;
    }

    const double requestedSpeed = data->jogSpeedUserUnitsPerSecond.load();
    if (requestedSpeed <= 0.0 || requestedSpeed > Racer3RtTaskJogMaxSpeedUserUnitsPerSecond)
    {
        rejectJogMotion(data, sequence, Racer3JogRejectInvalidSpeed);
        return;
    }

    try
    {
        data->lastMotionAxis = axisIndex;
        data->lastMotionStepUserUnits = requestedStep;
        data->lastMotionSpeedUserUnitsPerSecond = requestedSpeed;
        axis->MoveRelative(
            requestedStep,
            requestedSpeed,
            Racer3RtTaskJogDefaultAcceleration,
            Racer3RtTaskJogDefaultDeceleration,
            Racer3RtTaskJogDefaultJerkPercent);
        data->lastMotionCommandIssued = true;
        data->lastMotionCommandDone = axis->MotionDoneGet();
        data->lastMotionCommandSequenceAccepted = sequence;
        data->lastMotionRejectCode = Racer3JogRejectNone;
    }
    catch (const std::exception&)
    {
        rejectJogMotion(data, sequence, Racer3JogRejectMotionApiError);
        recordTaskError(data, Racer3JogRejectMotionApiError);
    }
}
} // namespace

// Official RSI sample-compatible heartbeat.  This deliberately mirrors the
// published Hello RTTasks sample name and global name so we can validate DLL
// deployment/task dispatch before exercising any Racer3-specific RTTask code.
RACER3_RSI_TASK(Increment)
{
    data->counter += 1;
}

// Minimal Racer3 heartbeat task. This intentionally does not call
// RTMotionControllerGet, RTAxisGet, RTMultiAxisGet, or any motion/setup API.
// It separates task/DLL dispatch from controller-object sampling.
RACER3_RSI_TASK(Racer3BasicHeartbeat)
{
    data->basicHeartbeat += 1;
    data->heartbeat += 1;
}

// Initialize only the shared RTTask globals and sample existing controller state.
// This task intentionally does not call Abort, ClearFaults, AmpEnableSet, AxisAdd,
// rsiconfig, or any motion primitive. Racer3's persistent session still owns all
// current setup/enabling behavior until the no-motion RTTask bridge is proven.
void initializeRacer3Globals(GlobalData* data)
{
    data->initialized = false;
    data->metadataReady = false;
    data->runtimeReady = false;
    data->statusSamplerReady = false;
    data->counter = 0;
    data->basicHeartbeat = 0;
    data->heartbeat = 0;
    data->initializationCount += 1;
    data->lastSampleCounter = 0;
    data->lastNetworkCounter = 0;
    data->statusSampleCount = 0;
    data->samplePeriodSeconds = 0.001;
    data->lastInitializationStep = Racer3ReadinessStepGlobalsReset;
    data->lastStatusSamplerStep = Racer3ReadinessStepNotStarted;
    data->lastStatusReadinessCode = Racer3ReadinessStepNotStarted;

    data->jogEnabled = false;
    data->jogStopRequested = false;
    data->jogMotionArmed = false;
    data->jogCommandSequence = 0;
    data->jogDirectionCode = 0;
    data->jogTargetAxis = 5;
    data->jogSpeedMetersPerSecond = 0.0;
    data->jogSpeedUserUnitsPerSecond = 0.0;
    data->jogStepUserUnits = 0.0;
    data->lastJogCommandSequenceSeen = 0;
    data->jogIntentTransitions = 0;
    data->lastJogStopSeen = 0;
    data->lastMotionCommandSequenceAccepted = 0;
    data->lastMotionCommandSequenceRejected = 0;
    data->lastMotionRejectCode = 0;
    data->lastMotionCommandIssued = false;
    data->lastMotionCommandDone = false;
    data->lastMotionAxis = -1;
    data->lastMotionStepUserUnits = 0.0;
    data->lastMotionSpeedUserUnitsPerSecond = 0.0;

    data->multiAxisReady = false;
    data->multiAxisAmpEnabled = false;
    data->multiAxisMotionDone = false;
    data->multiAxisState = 0;
    data->multiAxisMotionId = 0;
    data->multiAxisExecutingMotionId = 0;
    data->allAxisAmpEnabled = false;
    data->allAxisMotionDone = false;

    data->axis0CommandPosition = 0.0;
    data->axis1CommandPosition = 0.0;
    data->axis2CommandPosition = 0.0;
    data->axis3CommandPosition = 0.0;
    data->axis4CommandPosition = 0.0;
    data->axis5CommandPosition = 0.0;
    data->axis0ActualPosition = 0.0;
    data->axis1ActualPosition = 0.0;
    data->axis2ActualPosition = 0.0;
    data->axis3ActualPosition = 0.0;
    data->axis4ActualPosition = 0.0;
    data->axis5ActualPosition = 0.0;

    data->taskErrorCount = 0;
    data->lastErrorCode = 0;
    data->metadataReady = true;

    try
    {
        sampleRacer3MotionState(data, data->lastInitializationStep);
    }
    catch (const std::exception&)
    {
        recordReadinessError(data, data->lastInitializationStep);
        throw;
    }
}

// The laser demo and RSI helper samples use FunctionName=Initialize for the
// one-shot object/global setup task. Keep Racer3Initialize for the dynamic probe
// history, but export Initialize as the primary laser-style task-manager entry.
RACER3_RSI_TASK(Initialize)
{
    initializeRacer3Globals(data);
}

RACER3_RSI_TASK(Racer3Initialize)
{
    initializeRacer3Globals(data);
}

// Cyclic no-motion heartbeat/status task. Run this at 1-10 ms while validating
// RTTask manager setup. It proves the RTTask DLL can read the same controller,
// MultiAxis 6, and Axis 0..5 objects as the host process without touching amps.
RACER3_RSI_TASK(Racer3StatusSampler)
{
    try
    {
        data->heartbeat += 1;
        sampleRacer3MotionState(data, data->lastStatusSamplerStep);
    }
    catch (const std::exception&)
    {
        recordReadinessError(data, data->lastStatusSamplerStep);
    }
}

// Cyclic no-motion jog-intent monitor. Future patches can write jogEnabled,
// jogDirectionCode, jogSpeedMetersPerSecond, and jogCommandSequence from the
// host/UI. This task currently only records that the intent reached RTTask code.
RACER3_RSI_TASK(Racer3JogIntentMonitor)
{
    try
    {
        const int64_t sequence = data->jogCommandSequence.load();
        const int64_t previousSequence = data->lastJogCommandSequenceSeen.load();
        if (sequence != previousSequence)
        {
            data->lastJogCommandSequenceSeen = sequence;
            data->jogIntentTransitions += 1;
            issueTinyJogStepIfArmed(data, sequence);
        }
        else if (data->jogStopRequested)
        {
            data->lastJogStopSeen = sequence;
        }

        if (data->initialized)
        {
            sampleRacer3MotionState(data, data->lastStatusSamplerStep);
        }
    }
    catch (const std::exception&)
    {
        recordReadinessError(data, data->lastStatusSamplerStep);
    }
}
