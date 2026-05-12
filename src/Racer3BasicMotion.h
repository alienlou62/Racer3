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
    void connectController();
    void configureAxes();
    void configureMultiAxisMotionAttributes(const char* context);
    void configureAxis6MotionAttributes(const char* context);
    void isolateAxis6ForDirectMotion();
    void clearFaults();
    void enableAmplifiers();
    void enableOnlyTest();
    void runTinyMotion();
    void printMotionPlan() const;
    void printActualPositions(const char* label);
    void printDiagnosticSnapshot(const char* label, bool includeErrorLogs = true);
    void printMotionObjectDiagnostics(const char* label, RSI::RapidCode::RapidCodeMotion* motion);
    void printMotionAttributeMasks(const char* label, RSI::RapidCode::RapidCodeMotion* motion);
    void printAllAxisBriefDiagnostics(const char* label);
    void printAxis6MotionStatus(const char* label);
    void printMotionProgressLine(const char* label, int sampleNumber);
    void waitForMotionStart(const char* label, double startingAxis6CommandPosition);
    void waitForAxis6MotionStart(const char* label, double startingAxis6CommandPosition);
    void disableAmplifiers();
    void waitForMotionDone(int timeoutMilliseconds);
    void waitForAxis6MotionDone(int timeoutMilliseconds);
    void safeShutdown() noexcept;

    RSI::RapidCode::MotionController* controller_;
    RSI::RapidCode::MultiAxis* multiAxis_;
    std::array<RSI::RapidCode::Axis*, AxisCount> axes_;
};
