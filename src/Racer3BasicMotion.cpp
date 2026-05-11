#include "Racer3BasicMotion.h"

#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <rsi.h>
#include <stdexcept>
#include <string>

namespace RR = RSI::RapidCode;

static const std::array<std::array<double, Racer3BasicMotion::AxisCount>, 4> DemoOffsets = {
    std::array<double, Racer3BasicMotion::AxisCount>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    std::array<double, Racer3BasicMotion::AxisCount>{5.0, -5.0, 5.0, 5.0, -5.0, 5.0},
    std::array<double, Racer3BasicMotion::AxisCount>{-10.0, 10.0, -10.0, -10.0, 10.0, -10.0},
    std::array<double, Racer3BasicMotion::AxisCount>{5.0, -5.0, 5.0, 5.0, -5.0, 5.0}
};

static constexpr double DemoVelocity = 5.0;
static constexpr double DemoAcceleration = 10.0;
static constexpr double DemoDeceleration = 10.0;
static constexpr double DemoJerkPercent = 10.0;
static constexpr int AmpEnableTimeoutMs = 10000;
static constexpr int MotionTimeoutMs = 20000;

Racer3BasicMotion::Racer3BasicMotion()
    : controller_(nullptr), multiAxis_(nullptr), axes_{}
{
}

Racer3BasicMotion::~Racer3BasicMotion()
{
    safeShutdown();
}

void Racer3BasicMotion::run()
{
    connectController();
    clearFaults();
    enableAmplifiers();
    runDemoMotion();
    disableAmplifiers();
    safeShutdown();
}

void Racer3BasicMotion::connectController()
{
    std::cout << "Connecting to RSI MotionController...\n";
    controller_ = RR::MotionController::Create();
    if (!controller_)
    {
        throw std::runtime_error("Failed to create MotionController.");
    }

    std::cout << "Creating/initializing MultiAxis group...\n";
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
}

void Racer3BasicMotion::enableAmplifiers()
{
    std::cout << "Enabling amplifiers...\n";
    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }
    const int result = multiAxis_->AmpEnableSet(true, AmpEnableTimeoutMs, false);
    if (result == 0)
    {
        throw std::runtime_error("AmpEnableSet failed or timed out.");
    }
}

void Racer3BasicMotion::runDemoMotion()
{
    std::cout << "Starting demo motion sequence...\n";

    multiAxis_->VectorVelocitySet(DemoVelocity);
    multiAxis_->VectorAccelerationSet(DemoAcceleration);
    multiAxis_->VectorDecelerationSet(DemoDeceleration);
    multiAxis_->VectorJerkPercentSet(DemoJerkPercent);

    for (size_t stepIndex = 0; stepIndex < DemoOffsets.size(); ++stepIndex)
    {
        const auto& offset = DemoOffsets[stepIndex];
        std::cout << "Motion step " << (stepIndex + 1) << " / " << DemoOffsets.size() << "\n";
        multiAxis_->MoveVectorRelative(offset.data());
        waitForMotionDone(MotionTimeoutMs);
    }
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
