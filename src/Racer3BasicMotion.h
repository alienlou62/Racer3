#pragma once

#include <array>

namespace RSI
{
namespace RapidCode
{
class Axis;
class MotionController;
class MultiAxis;
class RapidCodeMotion;
}
}

struct Racer3RunOptions
{
    bool dryRun = false;
    bool enableOnly = false;
    bool tinyMotion = false;
    bool dualMotion = false;
    bool allMotion = false;
    bool motionConfirmed = false;
    bool diagnostics = false;
    double stepUserUnits = 0.05;
    double velocityUserUnitsPerSecond = 0.05;
};

class Racer3BasicMotion
{
public:
    static constexpr int AxisCount = 6;

    Racer3BasicMotion();
    ~Racer3BasicMotion();

    void run(const Racer3RunOptions& options);

private:
    void connectController();
    void configureAxes();
    void configureMultiAxisMotionAttributes(const char* context);
    void configureAxis6MotionAttributes(const char* context);
    void configureAxis5MotionAttributes(const char* context);
    void configureAxisMotionAttributes(int axisIndex, const char* context);
    void configureAllAxesForAllMotion();
    void isolateAxis6ForDirectMotion();
    void isolateAxis5And6ForDualMotion();
    void isolateAllAxesForAllMotion();
    void clearFaults();
    void enableAmplifiers();
    void enableOnlyTest();
    void runTinyMotion();
    void runDualAxisMotion();
    void runAllAxisMotion();
    void printMotionPlan() const;
    void printActualPositions(const char* label);
    void printDiagnosticSnapshot(const char* label, bool includeErrorLogs = true);
    void printMotionObjectDiagnostics(const char* label, RSI::RapidCode::RapidCodeMotion* motion);
    void printMotionAttributeMasks(const char* label, RSI::RapidCode::RapidCodeMotion* motion);
    void printAllAxisBriefDiagnostics(const char* label);
    void printAxis6MotionStatus(const char* label);
    void printAxis5And6MotionStatus(const char* label);
    void printAllAxisMotionStatus(const char* label);
    void printMotionProgressLine(const char* label, int sampleNumber);
    void printDualAxisProgressLine(const char* label, int sampleNumber);
    void printAllAxisProgressLine(const char* label, int sampleNumber);
    void waitForMotionStart(const char* label, double startingAxis6CommandPosition);
    void waitForAxis6MotionStart(const char* label, double startingAxis6CommandPosition);
    void waitForDualAxisMotionStart(const char* label, double startingAxis5CommandPosition, double startingAxis6CommandPosition);
    void waitForAllAxisMotionStart(const char* label, const std::array<double, AxisCount>& startingCommandPositions);
    void disableAmplifiers();
    void waitForMotionDone(int timeoutMilliseconds);
    void waitForAxis6MotionDone(int timeoutMilliseconds);
    void safeShutdown() noexcept;

    RSI::RapidCode::MotionController* controller_;
    RSI::RapidCode::MultiAxis* multiAxis_;
    std::array<RSI::RapidCode::Axis*, AxisCount> axes_;
};
