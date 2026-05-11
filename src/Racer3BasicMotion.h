#pragma once

#include <array>
#include <cstdint>

namespace RSI
{
namespace RapidCode
{
    class MotionController;
    class MultiAxis;
    class Axis;
    class RsiError;
}
}

struct Racer3RunOptions
{
    bool dryRun = false;
    bool enableOnly = true;
    bool tinyMotion = false;
    bool motionConfirmed = false;
};

class Racer3BasicMotion
{
public:
    static constexpr int AxisCount = 6;

    Racer3BasicMotion();
    ~Racer3BasicMotion();

    void run(const Racer3RunOptions& options);

private:
    RSI::RapidCode::MotionController* controller_;
    RSI::RapidCode::MultiAxis* multiAxis_;
    std::array<RSI::RapidCode::Axis*, AxisCount> axes_;

    void connectController();
    void configureAxes();
    void clearFaults();
    void enableAmplifiers();
    void enableOnlyTest();
    void runTinyMotion();
    void printMotionPlan() const;
    void printActualPositions(const char* label);
    void disableAmplifiers();
    void safeShutdown() noexcept;
    void waitForMotionDone(int timeoutMilliseconds);
};
