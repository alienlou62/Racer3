#include "Racer3BasicMotion.h"

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <rsi.h>
#include <stdexcept>
#include <string>
#include <thread>

namespace RR = RSI::RapidCode;

namespace
{
using JointVector = std::array<double, Racer3BasicMotion::AxisCount>;

// Baseline enable-only demo restored from the version that got past RMP startup.
// IMPORTANT: Do not run --tiny-motion from this baseline yet.
// The motion values below are still the old placeholder values.
const std::array<JointVector, 4> TinyMotionOffsets = {
    JointVector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    JointVector{0.25, -0.25, 0.25, 0.25, -0.25, 0.25},
    JointVector{-0.50, 0.50, -0.50, -0.50, 0.50, -0.50},
    JointVector{0.25, -0.25, 0.25, 0.25, -0.25, 0.25}
};

static constexpr double TinyVelocity = 0.5;
static constexpr double TinyAcceleration = 1.0;
static constexpr double TinyDeceleration = 1.0;
static constexpr double TinyJerkPercent = 5.0;

static constexpr int AmpEnableTimeoutMs = 10000;
static constexpr int MotionTimeoutMs = 20000;
static constexpr int EnableOnlyHoldSeconds = 2;
static constexpr int FaultClearSettleMs = 500;

// Current issue fixed by this file:
// After rsiconfig starts/configures RMP, AmpEnableSet(true, ..., false)
// can fail in RSIStateSTOPPED. The RapidCode error says to use
// overrideRestrictedState or ClearFaults(). We already call ClearFaults(),
// so this test enables with overrideRestrictedState = true.
static constexpr bool OverrideRestrictedStateForEnable = true;

void printJointVector(const JointVector& values)
{
    std::cout << std::fixed << std::setprecision(3);
    for (double value : values)
    {
        std::cout << std::setw(8) << value << ' ';
    }
    std::cout << '\n';
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

    connectController();
    clearFaults();
    printActualPositions("Actual positions before amp enable");
    enableAmplifiers();
    printActualPositions("Actual positions after amp enable");

    if (options.enableOnly)
    {
        enableOnlyTest();
    }
    else if (options.tinyMotion)
    {
        runTinyMotion();
    }

    printActualPositions("Actual positions before shutdown");
    disableAmplifiers();
    safeShutdown();
}

void Racer3BasicMotion::connectController()
{
    std::cout << "Connecting to RSI MotionController...\n";

    controller_ = RR::MotionController::Create();
    if (!controller_)
    {
        throw std::runtime_error("Failed to create MotionController. Confirm RMP has been started/configured with rsiconfig or RapidSetup.");
    }

    configureAxes();
}

void Racer3BasicMotion::configureAxes()
{
    std::cout << "Creating/initializing Racer3 6-axis MultiAxis group...\n";

    multiAxis_ = controller_->MultiAxisGet(0);
    if (!multiAxis_)
    {
        throw std::runtime_error("Failed to get MultiAxis object.");
    }

    for (int index = 0; index < AxisCount; ++index)
    {
        axes_[index] = controller_->AxisGet(index);
        if (!axes_[index])
        {
            throw std::runtime_error("Failed to get Axis " + std::to_string(index));
        }
    }

    multiAxis_->AxisRemoveAll();
    multiAxis_->AxesAdd(axes_.data(), AxisCount);
    multiAxis_->UserLabelSet("Racer3BasicMotion");
}

void Racer3BasicMotion::clearFaults()
{
    std::cout << "Clearing faults...\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    multiAxis_->ClearFaults();

    std::cout << "Fault clear command sent. Waiting briefly before amp enable...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(FaultClearSettleMs));
}

void Racer3BasicMotion::enableAmplifiers()
{
    std::cout << "Enabling amplifiers...\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    std::cout << "Using overrideRestrictedState="
              << (OverrideRestrictedStateForEnable ? "true" : "false")
              << " for this enable-only STOPPED-state test.\n";

    const int result = multiAxis_->AmpEnableSet(
        true,
        AmpEnableTimeoutMs,
        OverrideRestrictedStateForEnable);

    if (result == 0)
    {
        throw std::runtime_error("AmpEnableSet failed or timed out.");
    }
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
    std::cout << "Starting tiny relative joint-space motion sequence...\n";
    std::cout << "WARNING: This baseline file still contains old placeholder motion values.\n";
    std::cout << "Do not use --tiny-motion from this file until the J6-only motion version is restored.\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    multiAxis_->VectorVelocitySet(TinyVelocity);
    multiAxis_->VectorAccelerationSet(TinyAcceleration);
    multiAxis_->VectorDecelerationSet(TinyDeceleration);
    multiAxis_->VectorJerkPercentSet(TinyJerkPercent);

    for (size_t stepIndex = 0; stepIndex < TinyMotionOffsets.size(); ++stepIndex)
    {
        const auto& offset = TinyMotionOffsets[stepIndex];

        std::cout << "Tiny motion step "
                  << (stepIndex + 1)
                  << " / "
                  << TinyMotionOffsets.size()
                  << ": ";

        printJointVector(offset);

        multiAxis_->MoveVectorRelative(offset.data());
        waitForMotionDone(MotionTimeoutMs);
    }
}

void Racer3BasicMotion::printMotionPlan() const
{
    std::cout << "\nBaseline enable-only motion plan display.\n";
    std::cout << "WARNING: motion values below are old placeholders. Use --enable-only only.\n";
    std::cout << "Planned tiny relative joint offsets: J1 J2 J3 J4 J5 J6\n";

    JointVector netOffset{};
    for (size_t stepIndex = 0; stepIndex < TinyMotionOffsets.size(); ++stepIndex)
    {
        std::cout << "  Step " << (stepIndex + 1) << ": ";
        printJointVector(TinyMotionOffsets[stepIndex]);

        for (size_t axis = 0; axis < netOffset.size(); ++axis)
        {
            netOffset[axis] += TinyMotionOffsets[stepIndex][axis];
        }
    }

    std::cout << "  Net relative offset after sequence: ";
    printJointVector(netOffset);

    std::cout << "  Velocity=" << TinyVelocity
              << ", Acceleration=" << TinyAcceleration
              << ", Deceleration=" << TinyDeceleration
              << ", JerkPercent=" << TinyJerkPercent << "\n\n";
}

void Racer3BasicMotion::printActualPositions(const char* label)
{
    std::cout << label << ": ";
    std::cout << std::fixed << std::setprecision(6);

    for (auto* axis : axes_)
    {
        if (axis)
        {
            std::cout << axis->ActualPositionGet() << ' ';
        }
        else
        {
            std::cout << "<null> ";
        }
    }

    std::cout << '\n';
}

void Racer3BasicMotion::disableAmplifiers()
{
    if (!multiAxis_)
    {
        return;
    }

    std::cout << "Disabling amplifiers...\n";
    multiAxis_->AmpEnableSet(false);
}

void Racer3BasicMotion::waitForMotionDone(int timeoutMilliseconds)
{
    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    const int32_t done = multiAxis_->MotionDoneWait(timeoutMilliseconds);
    if (done == 0)
    {
        throw std::runtime_error("Motion did not complete within the timeout period.");
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
                multiAxis_->AmpEnableSet(false);
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
