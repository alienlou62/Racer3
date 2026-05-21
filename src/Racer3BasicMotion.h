#pragma once

#include <array>
#include <vector>

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
    bool jointVectorMotion = false;
    bool robotModelProbe = false;
    bool robotPoseProbe = false;
    bool kinematicsDryRun = false;
    bool cartesianVectorMotion = false;
    bool cartesianTraceMotion = false;
    bool positionOnlyIk = false;
    bool compactMotion = false;
    bool appendMotion = false;
    bool trajectoryMotion = false;
    bool endpointOnlyMotion = false;
    bool segmentGoalMotion = false;
    bool motionConfirmed = false;
    bool diagnostics = false;
    double stepUserUnits = 0.05;
    double velocityUserUnitsPerSecond = 0.05;
    double returnWarnToleranceUserUnits = 0.00025;
    double returnFailToleranceUserUnits = 0.00100;
    std::array<double, 6> jointVectorUserUnits{};
    std::array<double, 6> cartesianVector{};
    std::vector<std::array<double, 6>> cartesianTraceWaypoints;
};

class Racer3BasicMotion
{
public:
    static constexpr int AxisCount = 6;

    Racer3BasicMotion();
    ~Racer3BasicMotion();

    void run(const Racer3RunOptions& options);
    void startArmedSession(double velocityUserUnitsPerSecond, bool diagnostics);
    void stopArmedSessionMotion();
    void startArmedSessionAxis6VelocityJog(double velocityUserUnitsPerSecond);
    void stopArmedSessionAxis6VelocityJog(const char* reason);
    void printArmedSessionPositionSnapshot(const char* label);
    void runArmedSessionTrace(
        const std::vector<std::array<double, AxisCount>>& waypoints,
        double velocityUserUnitsPerSecond,
        bool returnToZero);
    void shutdownArmedSession() noexcept;

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
    void clearFaultsAfterCompletedMotion(const char* context) noexcept;
    void enableAmplifiers();
    void enableOnlyTest();
    void runTinyMotion();
    void runDualAxisMotion();
    void runAllAxisMotion();
    void runJointVectorMotion();
    void runRobotModelProbe();
    void runRobotPoseProbe();
    void runKinematicsDryRun();
    void runCartesianVectorMotion();
    void runCartesianTraceMotion();
    void printMotionPlan() const;
    void printActualPositions(const char* label);
    void printDiagnosticSnapshot(const char* label, bool includeErrorLogs = true);
    void printMotionObjectDiagnostics(const char* label, RSI::RapidCode::RapidCodeMotion* motion);
    void printMotionAttributeMasks(const char* label, RSI::RapidCode::RapidCodeMotion* motion);
    void printAllAxisBriefDiagnostics(const char* label);
    void printAxis6MotionStatus(const char* label);
    void printAxis5And6MotionStatus(const char* label);
    void printAllAxisMotionStatus(const char* label);
    bool printReturnToZeroReport(const char* label, bool throwOnFail);
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
    bool armedSessionAxis6VelocityJogActive_;
    double armedSessionAxis6VelocityJogCommandUserUnitsPerSecond_;
};




