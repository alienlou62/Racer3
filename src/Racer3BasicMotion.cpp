#include "Racer3BasicMotion.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <rsi.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace RR = RSI::RapidCode;

namespace
{
using JointVector = std::array<double, Racer3BasicMotion::AxisCount>;

static constexpr int MultiAxisIndex = 6;
static constexpr int Axis6Index = 5;

// Axis 6 / J6 counts per physical joint revolution from your Racer3 sheet.
static constexpr double Axis6CountsPerRevolution = 41943040.0;

// J6-only test.
// 1.0 user unit = one physical joint revolution.
// 0.05 user units = 18 degrees.
static constexpr double Axis6TestStepUserUnits = 0.05;

// Direct J6 MoveRelative values. The working hardware sequence enables all six
// drives through MultiAxis 6, removes the MultiAxis mapping, then commands only
// Axis 6 with Axis::MoveRelative(relativePosition, vel, accel, decel, jerkPct).
static constexpr double MotionVelocity = 0.05;
static constexpr double MotionAcceleration = 1.0;
static constexpr double MotionDeceleration = 1.0;
static constexpr double MotionJerkPercent = 5.0;

// After UserUnitsSet(41943040), these values are in revolutions/user-units.
static constexpr double Axis6FineTolerance = 0.001;     // 0.36 degrees
static constexpr double Axis6CoarseTolerance = 0.005;   // 1.8 degrees
static constexpr double Axis6VelocityTolerance = 0.001; // 0.36 deg/sec
static constexpr double Axis6SettlingTime = 0.05;       // seconds

static constexpr int AmpEnableTimeoutMs = 10000;
static constexpr int MotionTimeoutMs = 60000;
static constexpr int EnableOnlyHoldSeconds = 2;
static constexpr int FaultClearSettleMs = 500;
static constexpr int EnableSettleMs = 500;
static constexpr int MotionStatusSampleMs = 250;
static constexpr int MotionStartTimeoutMs = 3000;
static constexpr int MotionStartSampleMs = 50;

static constexpr bool OverrideRestrictedStateForEnable = true;
static constexpr bool TemporarilyDisableAxis6ErrorLimitForTinyMotion = true;

double toDegrees(double userUnits)
{
    return userUnits * 360.0;
}

void printJointVector(const JointVector& values)
{
    std::cout << std::fixed << std::setprecision(6);

    for (double value : values)
    {
        std::cout << std::setw(11) << value << ' ';
    }

    std::cout << '\n';
}

JointVector makeAxis6OnlyVector(double axis6Value)
{
    JointVector values{};
    values[Axis6Index] = axis6Value;
    return values;
}

JointVector makeAllAxesVector(double value)
{
    JointVector values{};
    values.fill(value);
    return values;
}

std::string hex64(uint64_t value)
{
    std::ostringstream stream;
    stream << "0x" << std::uppercase << std::hex << value;
    return stream.str();
}

const char* boolText(bool value)
{
    return value ? "true" : "false";
}

const char* stateName(RR::RSIState state)
{
    switch (state)
    {
    case RR::RSIState::RSIStateIDLE:
        return "IDLE";
    case RR::RSIState::RSIStateMOVING:
        return "MOVING";
    case RR::RSIState::RSIStateSTOPPING:
        return "STOPPING";
    case RR::RSIState::RSIStateSTOPPED:
        return "STOPPED";
    case RR::RSIState::RSIStateSTOPPING_ERROR:
        return "STOPPING_ERROR";
    case RR::RSIState::RSIStateERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

const char* actionName(RR::RSIAction action)
{
    switch (action)
    {
    case RR::RSIAction::RSIActionNONE:
        return "RSIActionNONE";
    case RR::RSIAction::RSIActionSTOP:
        return "RSIActionSTOP";
    case RR::RSIAction::RSIActionABORT:
        return "RSIActionABORT";
    case RR::RSIAction::RSIActionE_STOP:
        return "RSIActionE_STOP";
    default:
        return "RSIActionUNKNOWN";
    }
}

template <typename RapidCodeObjectT>
void printErrorLog(const char* label, RapidCodeObjectT* object)
{
    std::cout << label << " error log:\n";

    if (!object)
    {
        std::cout << "  <null object>\n";
        return;
    }

    int count = 0;

    try
    {
        count = object->ErrorLogCountGet();
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "  ErrorLogCountGet threw RapidCode error: " << error.text << "\n";
        std::cout << "  Function: " << error.functionName << "\n";
        return;
    }
    catch (...)
    {
        std::cout << "  ErrorLogCountGet threw unknown exception.\n";
        return;
    }

    std::cout << "  Count=" << count << "\n";

    for (int index = 0; index < count; ++index)
    {
        try
        {
            const RR::RsiError* const error = object->ErrorLogGet();

            if (!error)
            {
                std::cout << "  [" << index << "] <null error>\n";
                continue;
            }

            std::cout << "  [" << index << "] " << error->text << "\n";
            std::cout << "      Function: " << error->functionName << "\n";
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  [" << index << "] ErrorLogGet threw: " << error.text << "\n";
            std::cout << "      Function: " << error.functionName << "\n";
        }
        catch (...)
        {
            std::cout << "  [" << index << "] ErrorLogGet threw unknown exception.\n";
        }
    }
}

template <typename RapidCodeObjectT>
void clearErrorLog(const char* label, RapidCodeObjectT* object)
{
    if (!object)
    {
        return;
    }

    try
    {
        object->ErrorLogClear();
        std::cout << label << " error log cleared.\n";
    }
    catch (...)
    {
        std::cout << label << " error log clear failed.\n";
    }
}
}

Racer3BasicMotion::Racer3BasicMotion()
    : controller_(nullptr), multiAxis_(nullptr), axes_{}
{
}

Racer3BasicMotion::~Racer3BasicMotion()
{
    safeShutdown();
}

void Racer3BasicMotion::run(const Racer3RunOptions& options)
{
    printMotionPlan();

    if (options.dryRun)
    {
        std::cout << "Dry run complete. No RMP calls were made.\n";
        return;
    }

    try
    {
        connectController();

        clearFaults();

        printActualPositions("Actual positions after J6 software zero, before amp enable");
        printDiagnosticSnapshot("After clear faults, before amp enable");

        enableAmplifiers();

        printActualPositions("Actual positions after amp enable");
        printDiagnosticSnapshot("After amp enable");

        if (options.enableOnly)
        {
            enableOnlyTest();
        }
        else if (options.tinyMotion)
        {
            runTinyMotion();
        }

        printActualPositions("Actual positions before shutdown");
        printDiagnosticSnapshot("Before shutdown");

        disableAmplifiers();
        safeShutdown();
    }
    catch (...)
    {
        std::cerr << "Error path reached. Printing final diagnostics, then disabling amplifiers...\n";
        printDiagnosticSnapshot("Error path final diagnostic snapshot");
        disableAmplifiers();
        safeShutdown();
        throw;
    }
}

void Racer3BasicMotion::connectController()
{
    std::cout << "Connecting to RSI MotionController...\n";

    controller_ = RR::MotionController::Create();
    if (!controller_)
    {
        throw std::runtime_error(
            "Failed to create MotionController. Start/configure RMP first using scripts/start-racer3-rmp-and-run.ps1.");
    }

    configureAxes();
}

void Racer3BasicMotion::configureAxes()
{
    std::cout << "Getting Axis 0..5 individually...\n";

    for (int index = 0; index < AxisCount; ++index)
    {
        axes_[index] = controller_->AxisGet(index);
        if (!axes_[index])
        {
            throw std::runtime_error("Failed to get Axis " + std::to_string(index));
        }
    }

    std::cout << "Loading existing Racer3 MultiAxis at motion supervisor "
              << MultiAxisIndex
              << "...\n";

    // rsiconfig loads config/MultiAxis.xml into motion supervisor 6 before this
    // process starts. LoadExistingMultiAxis matches the RapidSetup sample path
    // for an already-created MultiAxis object.
    multiAxis_ = controller_->LoadExistingMultiAxis(MultiAxisIndex);
    if (!multiAxis_)
    {
        throw std::runtime_error("Failed to load existing MultiAxis object at index 6.");
    }

    std::cout << "Mapping Axis 0..5 into MultiAxis 6 at runtime, one axis at a time...\n";
    multiAxis_->AxisRemoveAll();

    for (int index = 0; index < AxisCount; ++index)
    {
        multiAxis_->AxisAdd(axes_[index]);
        std::cout << "  Added Axis " << (index + 1) << " (RapidCode index " << index << ")\n";
    }

    multiAxis_->UserLabelSet("Racer3J6Demo");
    configureMultiAxisMotionAttributes("after runtime AxisAdd mapping");

    std::cout << "Configuring ONLY Axis 6 / J6 for one-revolution user units...\n";

    if (!axes_[Axis6Index])
    {
        throw std::runtime_error("Axis 6 / J6 is not initialized.");
    }

    axes_[Axis6Index]->UserUnitsSet(Axis6CountsPerRevolution);
    axes_[Axis6Index]->PositionSet(0.0);

    // This matches setting Home Action = None in RapidSetup for Axis 6. It
    // removes the home/capture source as a blocker without changing amp or
    // hardware-limit safety actions.
    axes_[Axis6Index]->HomeActionSet(RR::RSIAction::RSIActionNONE);

    axes_[Axis6Index]->PositionToleranceFineSet(Axis6FineTolerance);
    axes_[Axis6Index]->PositionToleranceCoarseSet(Axis6CoarseTolerance);
    axes_[Axis6Index]->VelocityToleranceSet(Axis6VelocityTolerance);
    axes_[Axis6Index]->SettlingTimeSet(Axis6SettlingTime);

    std::cout << "Axis 6 / J6 user units set: 1.0 user unit = 1 physical revolution.\n";
    std::cout << "Axis 6 / J6 current position set to software zero.\n";
    std::cout << "Axis 6 / J6 HomeActionSet(RSIActionNONE) applied.\n";
    std::cout << "MultiAxis is loaded from rsiconfig XML, then runtime-mapped with AxisAdd.\n";
    std::cout << "MultiAxis motion attributes are reset to defaults with SYNC_START explicitly on.\n";
    std::cout << "Tiny-motion uses direct Axis 6::MoveRelative after MultiAxis 6 is unmapped.\n";
    std::cout << "  Axis 6 velocity=" << MotionVelocity << "\n";
    std::cout << "  Axis 6 acceleration=" << MotionAcceleration << "\n";
    std::cout << "  Axis 6 deceleration=" << MotionDeceleration << "\n";
    std::cout << "  Axis 6 jerkPercent=" << MotionJerkPercent << "\n";
    std::cout << "Axis 6 / J6 settling configured:\n";
    std::cout << "  Fine tolerance: " << Axis6FineTolerance
              << " user units = " << toDegrees(Axis6FineTolerance) << " degrees\n";
    std::cout << "  Coarse tolerance: " << Axis6CoarseTolerance
              << " user units = " << toDegrees(Axis6CoarseTolerance) << " degrees\n";
    std::cout << "  Velocity tolerance: " << Axis6VelocityTolerance
              << " user-units/sec = " << toDegrees(Axis6VelocityTolerance) << " deg/sec\n";
    std::cout << "  Settling time: " << Axis6SettlingTime << " seconds\n";

    printDiagnosticSnapshot("After configureAxes and runtime MultiAxis mapping");
}

void Racer3BasicMotion::configureMultiAxisMotionAttributes(const char* context)
{
    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    std::cout << "Resetting MultiAxis motion attributes (" << context << ")...\n";

    // RapidCode motion attributes are sticky on the motion object. Resetting to
    // defaults clears stale HOLD/DELAY/APPEND/NO_WAIT/NO_HANDSHAKE state from a
    // previous session, then SYNC_START is explicitly asserted to match the
    // official MultiAxis sample pattern.
    multiAxis_->MotionAttributeMaskDefaultSet();
    multiAxis_->MotionAttributeMaskOnSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskSYNC_START);
    multiAxis_->MotionDelaySet(0.0);
    multiAxis_->FeedRateSet(1.0);

    printMotionAttributeMasks("MultiAxis 6", multiAxis_);
}

void Racer3BasicMotion::configureAxis6MotionAttributes(const char* context)
{
    if (!axes_[Axis6Index])
    {
        throw std::runtime_error("Axis 6 / J6 is not initialized.");
    }

    std::cout << "Resetting Axis 6 motion attributes (" << context << ")...\n";

    // Hardware tests showed six-axis MultiAxis motion is stopped by non-J6 axes
    // that still report Home/Capture Status Limit. For the J6-only test, keep
    // those axes enabled but do not include them in the commanded motion path.
    axes_[Axis6Index]->MotionAttributeMaskDefaultSet();
    axes_[Axis6Index]->MotionDelaySet(0.0);
    axes_[Axis6Index]->FeedRateSet(1.0);

    printMotionAttributeMasks("Axis 6", axes_[Axis6Index]);
}

void Racer3BasicMotion::isolateAxis6ForDirectMotion()
{
    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    if (!axes_[Axis6Index])
    {
        throw std::runtime_error("Axis 6 / J6 is not initialized.");
    }

    std::cout << "Isolating Axis 6 for direct motion by removing all axes from MultiAxis 6...\n";
    std::cout << "All six drives have already been enabled; cleanup now disables axes individually as well.\n";

    // A mapped Axis still propagates direct Axis::MoveRelative through its
    // MultiAxis group. Removing the group mapping lets this test command only
    // J6 without the Home/Capture stop source from axes 1-5.
    multiAxis_->AxisRemoveAll();

    std::this_thread::sleep_for(std::chrono::milliseconds(EnableSettleMs));

    configureAxis6MotionAttributes("after MultiAxis AxisRemoveAll");

    std::cout << "Clearing Axis 6 faults and confirming Axis 6 amp enable directly...\n";
    axes_[Axis6Index]->ClearFaults();

    const int result = axes_[Axis6Index]->AmpEnableSet(
        true,
        AmpEnableTimeoutMs,
        OverrideRestrictedStateForEnable);

    if (!axes_[Axis6Index]->AmpEnableGet())
    {
        throw std::runtime_error("Axis 6 AmpEnableSet failed or timed out after isolation.");
    }

    std::cout << "Axis 6 amp enable confirmed after isolation. AmpEnableSet returned "
              << result
              << " ms.\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(EnableSettleMs));
    printDiagnosticSnapshot("After isolating Axis 6 for direct motion");
}

void Racer3BasicMotion::clearFaults()
{
    std::cout << "Aborting and clearing faults through MultiAxis 6...\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    multiAxis_->Abort();
    multiAxis_->ClearFaults();

    std::cout << "Abort + fault clear sent. Waiting briefly before amp enable...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(FaultClearSettleMs));
}

void Racer3BasicMotion::enableAmplifiers()
{
    std::cout << "Enabling amplifiers through MultiAxis 6...\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    std::cout << "Using overrideRestrictedState="
              << (OverrideRestrictedStateForEnable ? "true" : "false")
              << ".\n";

    const int result = multiAxis_->AmpEnableSet(
        true,
        AmpEnableTimeoutMs,
        OverrideRestrictedStateForEnable);

    if (!multiAxis_->AmpEnableGet())
    {
        throw std::runtime_error("AmpEnableSet failed or timed out.");
    }

    std::cout << "Amplifier enable confirmed. AmpEnableSet returned "
              << result
              << " ms. Waiting briefly for drives to settle...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(EnableSettleMs));
}

void Racer3BasicMotion::enableOnlyTest()
{
    std::cout << "Enable-only test active. Holding enabled state for "
              << EnableOnlyHoldSeconds
              << " seconds...\n";

    std::this_thread::sleep_for(std::chrono::seconds(EnableOnlyHoldSeconds));

    std::cout << "Enable-only hold complete. No motion was commanded.\n";
}

void Racer3BasicMotion::runTinyMotion()
{
    std::cout << "Starting Axis 6 / J6 only direct Axis::MoveRelative diagnostic...\n";
    std::cout << "All 6 axes were enabled through runtime-mapped MultiAxis 6.\n";
    std::cout << "The actual tiny move is commanded only on Axis 6 / J6.\n";
    std::cout << "Before the J6 move, MultiAxis 6 is unmapped so axes 1-5 cannot stop the J6 trajectory.\n";
    std::cout << "Step = "
              << Axis6TestStepUserUnits
              << " user units = "
              << toDegrees(Axis6TestStepUserUnits)
              << " degrees.\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    if (!axes_[Axis6Index])
    {
        throw std::runtime_error("Axis 6 / J6 is not initialized.");
    }

    configureMultiAxisMotionAttributes("before tiny-motion command sequence");
    configureAxis6MotionAttributes("before tiny-motion command sequence");
    isolateAxis6ForDirectMotion();

    const RR::RSIAction originalAxis6ErrorLimitAction = axes_[Axis6Index]->ErrorLimitActionGet();
    bool axis6ErrorLimitTemporarilyChanged = false;

    if (TemporarilyDisableAxis6ErrorLimitForTinyMotion)
    {
        std::cout << "Temporarily setting Axis 6 position ErrorLimitAction to RSIActionNONE for this tiny motion test.\n";
        std::cout << "  Original Axis 6 ErrorLimitAction: " << actionName(originalAxis6ErrorLimitAction) << "\n";
        std::cout << "  Amp fault and hardware limit actions are not changed.\n";
        axes_[Axis6Index]->ErrorLimitActionSet(RR::RSIAction::RSIActionNONE);
        axis6ErrorLimitTemporarilyChanged = true;
    }

    const std::array<double, 2> relativePositions = {
        Axis6TestStepUserUnits,
        -Axis6TestStepUserUnits
    };

    try
    {
        for (size_t stepIndex = 0; stepIndex < relativePositions.size(); ++stepIndex)
        {
            const double relativePosition = relativePositions[stepIndex];

            std::cout << "\n=== Axis 6::MoveRelative step "
                      << (stepIndex + 1)
                      << " / "
                      << relativePositions.size()
                      << " ===\n";

            std::cout << "Axis 6 relative position: "
                      << relativePosition
                      << " user units = "
                      << toDegrees(relativePosition)
                      << " degrees.\n";
            std::cout << "Axis 6 velocity=" << MotionVelocity
                      << ", acceleration=" << MotionAcceleration
                      << ", deceleration=" << MotionDeceleration
                      << ", jerkPercent=" << MotionJerkPercent
                      << "\n";

            clearErrorLog("MotionController", controller_);
            clearErrorLog("MultiAxis 6", multiAxis_);
            clearErrorLog("Axis 6", axes_[Axis6Index]);

            configureAxis6MotionAttributes("before direct Axis 6::MoveRelative step");
            printDiagnosticSnapshot("Before direct Axis 6::MoveRelative");

            const uint16_t commandedMotionId = axes_[Axis6Index]->MotionIdGet();
            const double startingAxis6CommandPosition = axes_[Axis6Index]->CommandPositionGet();

            std::cout << "Commanding Axis 6::MoveRelative(relativePosition, velocity, acceleration, deceleration, jerk).\n";
            std::cout << "  Axis 6 commanded MotionId before call: " << commandedMotionId << "\n";

            axes_[Axis6Index]->MoveRelative(
                relativePosition,
                MotionVelocity,
                MotionAcceleration,
                MotionDeceleration,
                MotionJerkPercent);

            std::cout << "  Axis 6 next MotionId after call: " << axes_[Axis6Index]->MotionIdGet() << "\n";
            printDiagnosticSnapshot("Immediately after direct Axis 6::MoveRelative");

            waitForAxis6MotionStart("Axis 6::MoveRelative", startingAxis6CommandPosition);

            for (int sample = 0; sample < 8; ++sample)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(MotionStatusSampleMs));
                printMotionProgressLine("Live sample", sample + 1);
            }

            waitForAxis6MotionDone(MotionTimeoutMs);

            printActualPositions("Actual positions after direct Axis 6::MoveRelative step");
            printDiagnosticSnapshot("After Axis 6 MotionDoneWait");
        }
    }
    catch (...)
    {
        if (axis6ErrorLimitTemporarilyChanged)
        {
            axes_[Axis6Index]->ErrorLimitActionSet(originalAxis6ErrorLimitAction);
            std::cout << "Restored Axis 6 ErrorLimitAction to " << actionName(originalAxis6ErrorLimitAction) << ".\n";
        }

        throw;
    }

    if (axis6ErrorLimitTemporarilyChanged)
    {
        axes_[Axis6Index]->ErrorLimitActionSet(originalAxis6ErrorLimitAction);
        std::cout << "Restored Axis 6 ErrorLimitAction to " << actionName(originalAxis6ErrorLimitAction) << ".\n";
    }

    std::cout << "Axis 6 / J6 direct MoveRelative diagnostic complete. Net commanded Axis 6 offset is zero.\n";
}

void Racer3BasicMotion::printMotionPlan() const
{
    std::cout << "\nAxis 6 / J6 direct MoveRelative diagnostic motion plan\n";
    std::cout << "Startup path: scripts/start-racer3-rmp-and-run.ps1 runs rsiconfig first.\n";
    std::cout << "Enable path: LoadExistingMultiAxis(6), then AxisRemoveAll/AxisAdd, then enable all six drives.\n";
    std::cout << "Isolation path: after all six drives are enabled, MultiAxis 6 is unmapped with AxisRemoveAll.\n";
    std::cout << "Motion path: clear/enable Axis 6 directly, then Axis 6::MoveRelative(relativePosition, vel, accel, decel, jerkPct).\n";
    std::cout << "Reason: current logs show axes 1-5 stopped by Home / Capture Status Limit, which stops six-axis synchronized MultiAxis motion.\n";
    std::cout << "Axis 6 scaling: 1.0 user unit = 1 physical revolution.\n";
    std::cout << "Axis 6 HomeAction is set to NONE in code.\n";
    std::cout << "Axis 6 ErrorLimitAction is temporarily set to NONE only during tiny-motion.\n";
    std::cout << "Only J6 receives a motion command. Cleanup disables each axis individually.\n";
    std::cout << "Test step = "
              << Axis6TestStepUserUnits
              << " user units = "
              << toDegrees(Axis6TestStepUserUnits)
              << " degrees.\n";

    const std::array<JointVector, 2> plannedMoves = {
        makeAxis6OnlyVector(Axis6TestStepUserUnits),
        makeAxis6OnlyVector(-Axis6TestStepUserUnits)
    };

    JointVector netOffset{};

    for (size_t stepIndex = 0; stepIndex < plannedMoves.size(); ++stepIndex)
    {
        std::cout << "  Relative step " << (stepIndex + 1) << " user units: ";
        printJointVector(plannedMoves[stepIndex]);

        for (size_t axis = 0; axis < netOffset.size(); ++axis)
        {
            netOffset[axis] += plannedMoves[stepIndex][axis];
        }

        JointVector stepDegrees{};
        for (size_t axis = 0; axis < plannedMoves[stepIndex].size(); ++axis)
        {
            stepDegrees[axis] = toDegrees(plannedMoves[stepIndex][axis]);
        }

        std::cout << "                    approx deg: ";
        printJointVector(stepDegrees);
    }

    std::cout << "  Net relative offset after sequence in user units: ";
    printJointVector(netOffset);

    std::cout << "  MotionVelocity="
              << MotionVelocity
              << ", MotionAcceleration="
              << MotionAcceleration
              << ", MotionDeceleration="
              << MotionDeceleration
              << ", MotionJerkPercent="
              << MotionJerkPercent
              << "\n\n";
}

void Racer3BasicMotion::printActualPositions(const char* label)
{
    std::cout << label << ": ";
    std::cout << std::fixed << std::setprecision(6);

    for (int index = 0; index < AxisCount; ++index)
    {
        if (axes_[index])
        {
            std::cout << axes_[index]->ActualPositionGet() << ' ';
        }
        else
        {
            std::cout << "<null> ";
        }
    }

    std::cout << '\n';
}

void Racer3BasicMotion::printDiagnosticSnapshot(const char* label, bool includeErrorLogs)
{
    std::cout << "\n--- " << label << " ---\n";

    printMotionObjectDiagnostics("MultiAxis 6", multiAxis_);

    if (axes_[Axis6Index])
    {
        printMotionObjectDiagnostics("Axis 6", axes_[Axis6Index]);
        printAxis6MotionStatus("Axis 6 numeric status");
    }

    printAllAxisBriefDiagnostics("All-axis brief status");

    if (includeErrorLogs)
    {
        printErrorLog("MotionController", controller_);
        printErrorLog("MultiAxis 6", multiAxis_);
        printErrorLog("Axis 6", axes_[Axis6Index]);
    }
    else
    {
        std::cout << "Error logs omitted for this compact live sample.\n";
    }

    std::cout << "--- end diagnostic snapshot ---\n";
}

void Racer3BasicMotion::printMotionObjectDiagnostics(const char* label, RR::RapidCodeMotion* motion)
{
    std::cout << label << " diagnostics:\n";

    if (!motion)
    {
        std::cout << "  <null>\n";
        return;
    }

    try
    {
        const RR::RSIState state = motion->StateGet();
        const RR::RSISource source = motion->SourceGet();

        std::cout << "  StateName=" << stateName(state) << "\n";
        std::cout << "  StateRaw=" << static_cast<int>(state) << "\n";
        std::cout << "  SourceName=" << motion->SourceNameGet(source) << "\n";
        std::cout << "  StatusBits=" << hex64(motion->StatusBitsGet()) << "\n";
        std::cout << "  AxisCountGet=" << motion->AxisCountGet() << "\n";
        std::cout << "  AmpEnableGet=" << boolText(motion->AmpEnableGet()) << "\n";
        std::cout << "  MotionDoneGet=" << boolText(motion->MotionDoneGet()) << "\n";
        std::cout << "  IsMapped=" << boolText(motion->IsMapped()) << "\n";
        std::cout << "  FeedRate=" << motion->FeedRateGet() << "\n";
        std::cout << "  MotionDelay=" << motion->MotionDelayGet() << "\n";
        std::cout << "  MotionId=" << motion->MotionIdGet() << "\n";
        std::cout << "  MotionIdExecuting=" << motion->MotionIdExecutingGet() << "\n";
        std::cout << "  MotionElementIdExecuting=" << motion->MotionElementIdExecutingGet() << "\n";
        std::cout << "  MotionHoldGateNumber=" << motion->MotionHoldGateNumberGet() << "\n";
        std::cout << "  MotionHoldGate=" << boolText(motion->MotionHoldGateGet()) << "\n";
        std::cout << "  MotionHoldTimeout=" << motion->MotionHoldTimeoutGet() << "\n";
        std::cout << "  MotionHoldTypeRaw=" << static_cast<int>(motion->MotionHoldTypeGet()) << "\n";
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "  Diagnostic read threw RapidCode error: " << error.text << "\n";
        std::cout << "  Function: " << error.functionName << "\n";
    }
    catch (const std::exception& error)
    {
        std::cout << "  Diagnostic read threw std::exception: " << error.what() << "\n";
    }
    catch (...)
    {
        std::cout << "  Diagnostic read threw unknown exception.\n";
    }

    printMotionAttributeMasks(label, motion);
}

void Racer3BasicMotion::printMotionAttributeMasks(const char* label, RR::RapidCodeMotion* motion)
{
    struct MaskInfo
    {
        RR::RSIMotionAttrMask mask;
        const char* name;
    };

    static const MaskInfo masks[] = {
        {RR::RSIMotionAttrMask::RSIMotionAttrMaskRELATIVE, "RELATIVE"},
        {RR::RSIMotionAttrMask::RSIMotionAttrMaskHOLD, "HOLD"},
        {RR::RSIMotionAttrMask::RSIMotionAttrMaskDELAY, "DELAY"},
        {RR::RSIMotionAttrMask::RSIMotionAttrMaskAPPEND, "APPEND"},
        {RR::RSIMotionAttrMask::RSIMotionAttrMaskAUTO_START, "AUTO_START"},
        {RR::RSIMotionAttrMask::RSIMotionAttrMaskSYNC_START, "SYNC_START"},
        {RR::RSIMotionAttrMask::RSIMotionAttrMaskSYNC_END, "SYNC_END"},
        {RR::RSIMotionAttrMask::RSIMotionAttrMaskNO_HANDSHAKE, "NO_HANDSHAKE"},
        {RR::RSIMotionAttrMask::RSIMotionAttrMaskNO_WAIT, "NO_WAIT"}
    };

    std::cout << label << " motion attribute masks:\n";

    if (!motion)
    {
        std::cout << "  <null>\n";
        return;
    }

    try
    {
        for (const MaskInfo& mask : masks)
        {
            std::cout << "  " << mask.name << "="
                      << (motion->MotionAttributeMaskOnGet(mask.mask) ? "on" : "off")
                      << "\n";
        }
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "  MotionAttributeMaskOnGet threw RapidCode error: " << error.text << "\n";
        std::cout << "  Function: " << error.functionName << "\n";
    }
    catch (...)
    {
        std::cout << "  MotionAttributeMaskOnGet threw unknown exception.\n";
    }
}

void Racer3BasicMotion::printAllAxisBriefDiagnostics(const char* label)
{
    std::cout << label << ":\n";

    for (int index = 0; index < AxisCount; ++index)
    {
        RR::Axis* axis = axes_[index];
        std::cout << "  Axis " << (index + 1) << " (RapidCode index " << index << "): ";

        if (!axis)
        {
            std::cout << "<null>\n";
            continue;
        }

        try
        {
            const RR::RSIState state = axis->StateGet();
            const RR::RSISource source = axis->SourceGet();

            std::cout << "State=" << stateName(state)
                      << " Source=" << axis->SourceNameGet(source)
                      << " Amp=" << boolText(axis->AmpEnableGet())
                      << " Mapped=" << boolText(axis->IsMapped())
                      << " Done=" << boolText(axis->MotionDoneGet())
                      << " MotionId=" << axis->MotionIdGet()
                      << " Exec=" << axis->MotionIdExecutingGet()
                      << " CmdPos=" << std::fixed << std::setprecision(6) << axis->CommandPositionGet()
                      << " CmdVel=" << axis->CommandVelocityGet()
                      << "\n";
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "diagnostic threw RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }
        catch (...)
        {
            std::cout << "diagnostic threw unknown exception.\n";
        }
    }
}

void Racer3BasicMotion::printAxis6MotionStatus(const char* label)
{
    if (!axes_[Axis6Index])
    {
        std::cout << label << ": Axis 6 not initialized.\n";
        return;
    }

    try
    {
        std::cout << label
                  << " | CmdPos=" << std::fixed << std::setprecision(6) << axes_[Axis6Index]->CommandPositionGet()
                  << " ActPos=" << axes_[Axis6Index]->ActualPositionGet()
                  << " TargetPos=" << axes_[Axis6Index]->TargetPositionGet()
                  << " PosErr=" << axes_[Axis6Index]->PositionErrorGet()
                  << " CmdVel=" << axes_[Axis6Index]->CommandVelocityGet()
                  << " ActVel=" << axes_[Axis6Index]->ActualVelocityGet()
                  << '\n';
    }
    catch (const RR::RsiError& error)
    {
        std::cout << label << " numeric status threw RapidCode error: " << error.text << "\n";
        std::cout << "Function: " << error.functionName << "\n";
    }
}

void Racer3BasicMotion::printMotionProgressLine(const char* label, int sampleNumber)
{
    if (!multiAxis_ || !axes_[Axis6Index])
    {
        std::cout << label << " sample " << sampleNumber << ": motion objects are not initialized.\n";
        return;
    }

    try
    {
        const RR::RSIState multiState = multiAxis_->StateGet();
        const RR::RSIState axisState = axes_[Axis6Index]->StateGet();

        std::cout << label
                  << " sample " << sampleNumber
                  << " | MultiAxis(State=" << stateName(multiState)
                  << ", MotionId=" << multiAxis_->MotionIdGet()
                  << ", Exec=" << multiAxis_->MotionIdExecutingGet()
                  << ", Done=" << boolText(multiAxis_->MotionDoneGet())
                  << ") Axis6(State=" << stateName(axisState)
                  << ", MotionId=" << axes_[Axis6Index]->MotionIdGet()
                  << ", Exec=" << axes_[Axis6Index]->MotionIdExecutingGet()
                  << ", Done=" << boolText(axes_[Axis6Index]->MotionDoneGet())
                  << ", CmdPos=" << std::fixed << std::setprecision(6) << axes_[Axis6Index]->CommandPositionGet()
                  << ", CmdVel=" << axes_[Axis6Index]->CommandVelocityGet()
                  << ", ActPos=" << axes_[Axis6Index]->ActualPositionGet()
                  << ", ActVel=" << axes_[Axis6Index]->ActualVelocityGet()
                  << ")\n";
    }
    catch (const RR::RsiError& error)
    {
        std::cout << label << " sample " << sampleNumber
                  << " threw RapidCode error: " << error.text
                  << " (" << error.functionName << ")\n";
    }
}

void Racer3BasicMotion::waitForMotionStart(const char* label, double startingAxis6CommandPosition)
{
    if (!multiAxis_ || !axes_[Axis6Index])
    {
        throw std::runtime_error("Motion objects are not initialized.");
    }

    std::cout << "Watching for " << label << " to generate an executing trajectory...\n";

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(MotionStartTimeoutMs);
    int sampleNumber = 0;

    while (std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(MotionStartSampleMs));
        ++sampleNumber;

        printMotionProgressLine("Start-watch", sampleNumber);

        const RR::RSIState multiState = multiAxis_->StateGet();
        const RR::RSIState axisState = axes_[Axis6Index]->StateGet();
        const double axis6CommandPosition = axes_[Axis6Index]->CommandPositionGet();
        const double axis6CommandVelocity = axes_[Axis6Index]->CommandVelocityGet();

        const bool stateMoving =
            multiState == RR::RSIState::RSIStateMOVING ||
            axisState == RR::RSIState::RSIStateMOVING;
        const bool commandPositionChanged =
            std::fabs(axis6CommandPosition - startingAxis6CommandPosition) > 1e-7;
        const bool commandVelocityNonZero = std::fabs(axis6CommandVelocity) > 1e-9;

        if (stateMoving || commandPositionChanged || commandVelocityNonZero)
        {
            std::cout << label << " started: "
                      << "stateMoving=" << boolText(stateMoving)
                      << ", commandPositionChanged=" << boolText(commandPositionChanged)
                      << ", commandVelocityNonZero=" << boolText(commandVelocityNonZero)
                      << ".\n";
            return;
        }
    }

    printDiagnosticSnapshot("Motion command accepted but no executing trajectory appeared");

    try
    {
        std::cout << "Aborting the accepted-but-not-started MultiAxis command before error exit...\n";
        multiAxis_->Abort();
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "Abort after non-started command threw RapidCode error: "
                  << error.text
                  << " ("
                  << error.functionName
                  << ")\n";
    }

    throw std::runtime_error(
        "MultiAxis command was accepted, but Axis 6 command position/velocity and motion state did not change.");
}

void Racer3BasicMotion::waitForAxis6MotionStart(const char* label, double startingAxis6CommandPosition)
{
    if (!axes_[Axis6Index])
    {
        throw std::runtime_error("Axis 6 / J6 is not initialized.");
    }

    std::cout << "Watching for " << label << " to generate an Axis 6 trajectory...\n";

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(MotionStartTimeoutMs);
    int sampleNumber = 0;

    while (std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(MotionStartSampleMs));
        ++sampleNumber;

        printMotionProgressLine("Axis6 start-watch", sampleNumber);

        const RR::RSIState axisState = axes_[Axis6Index]->StateGet();
        const double axis6CommandPosition = axes_[Axis6Index]->CommandPositionGet();
        const double axis6CommandVelocity = axes_[Axis6Index]->CommandVelocityGet();

        const bool stateMoving = axisState == RR::RSIState::RSIStateMOVING;
        const bool commandPositionChanged =
            std::fabs(axis6CommandPosition - startingAxis6CommandPosition) > 1e-7;
        const bool commandVelocityNonZero = std::fabs(axis6CommandVelocity) > 1e-9;

        if (stateMoving || commandPositionChanged || commandVelocityNonZero)
        {
            std::cout << label << " started: "
                      << "axisStateMoving=" << boolText(stateMoving)
                      << ", commandPositionChanged=" << boolText(commandPositionChanged)
                      << ", commandVelocityNonZero=" << boolText(commandVelocityNonZero)
                      << ".\n";
            return;
        }
    }

    printDiagnosticSnapshot("Axis 6 command accepted but no executing trajectory appeared");

    try
    {
        std::cout << "Aborting Axis 6 accepted-but-not-started command before error exit...\n";
        axes_[Axis6Index]->Abort();
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "Axis 6 abort after non-started command threw RapidCode error: "
                  << error.text
                  << " ("
                  << error.functionName
                  << ")\n";
    }

    throw std::runtime_error(
        "Axis 6 command was accepted, but command position/velocity and motion state did not change.");
}

void Racer3BasicMotion::disableAmplifiers()
{
    std::cout << "Disabling amplifiers...\n";

    if (multiAxis_)
    {
        try
        {
            multiAxis_->AmpEnableSet(false);
            std::cout << "  MultiAxis 6 amp disable command sent.\n";
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  MultiAxis 6 amp disable threw RapidCode error: "
                      << error.text
                      << " ("
                      << error.functionName
                      << ")\n";
        }
    }

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            continue;
        }

        try
        {
            axes_[index]->AmpEnableSet(false);
            std::cout << "  Axis " << (index + 1) << " amp disable command sent.\n";
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  Axis " << (index + 1)
                      << " amp disable threw RapidCode error: "
                      << error.text
                      << " ("
                      << error.functionName
                      << ")\n";
        }
    }
}

void Racer3BasicMotion::waitForMotionDone(int timeoutMilliseconds)
{
    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    std::cout << "Waiting for MultiAxis 6 motion done...\n";

    try
    {
        const int32_t elapsedMilliseconds = multiAxis_->MotionDoneWait(timeoutMilliseconds);
        std::cout << "MotionDoneWait returned after "
                  << elapsedMilliseconds
                  << " ms.\n";
    }
    catch (const RR::RsiError&)
    {
        printActualPositions("Actual positions at MultiAxis MotionDoneWait error");
        printDiagnosticSnapshot("At MultiAxis MotionDoneWait error");
        throw;
    }
}

void Racer3BasicMotion::waitForAxis6MotionDone(int timeoutMilliseconds)
{
    if (!axes_[Axis6Index])
    {
        throw std::runtime_error("Axis 6 / J6 is not initialized.");
    }

    std::cout << "Waiting for Axis 6 motion done...\n";

    try
    {
        const int32_t elapsedMilliseconds = axes_[Axis6Index]->MotionDoneWait(timeoutMilliseconds);
        std::cout << "Axis 6 MotionDoneWait returned after "
                  << elapsedMilliseconds
                  << " ms.\n";
    }
    catch (const RR::RsiError&)
    {
        printActualPositions("Actual positions at Axis 6 MotionDoneWait error");
        printDiagnosticSnapshot("At Axis 6 MotionDoneWait error");
        throw;
    }
}

void Racer3BasicMotion::safeShutdown() noexcept
{
    try
    {
        if (controller_)
        {
            if (multiAxis_)
            {
                try
                {
                    multiAxis_->AmpEnableSet(false);
                }
                catch (...)
                {
                }
            }

            for (auto* axis : axes_)
            {
                if (!axis)
                {
                    continue;
                }

                try
                {
                    axis->AmpEnableSet(false);
                }
                catch (...)
                {
                }
            }

            controller_->Delete();
            controller_ = nullptr;
            multiAxis_ = nullptr;
            axes_.fill(nullptr);
        }
    }
    catch (...)
    {
        // Best effort cleanup; do not throw from destructor.
    }
}
