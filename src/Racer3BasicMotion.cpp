#include "Racer3BasicMotion.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <iostream>
#include <rsi.h>
#include <cartesianrobot.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace RR = RSI::RapidCode;
namespace RC = RSI::RapidCode::Cartesian;

namespace
{
using JointVector = std::array<double, Racer3BasicMotion::AxisCount>;
using CartesianVector = std::array<double, Racer3BasicMotion::AxisCount>;

static constexpr int MultiAxisIndex = 6;
static constexpr int Axis5Index = 4;
static constexpr int Axis6Index = 5;

// Racer3 counts per physical joint revolution from your sheet.
// One RMP user unit = one full physical joint revolution for each axis.
const JointVector Racer3CountsPerRevolution = {
    83886080.0,  // Axis 1 / J1
    83886080.0,  // Axis 2 / J2
    67108864.0,  // Axis 3 / J3
    67108864.0,  // Axis 4 / J4
    67108864.0,  // Axis 5 / J5
    41943040.0   // Axis 6 / J6
};

static constexpr double Axis5CountsPerRevolution = 67108864.0;
static constexpr double Axis6CountsPerRevolution = 41943040.0;

// J6-only and J5+J6 dual-axis tests.
// 1.0 user unit = one physical joint revolution on each configured axis.
// 0.05 user units = 18 degrees.
static double Axis6TestStepUserUnits = 0.05;

// Direct J6 MoveRelative values. The working hardware sequence enables all six
// drives through MultiAxis 6, removes the MultiAxis mapping, then commands only
// Axis 6 with Axis::MoveRelative(relativePosition, vel, accel, decel, jerkPct).
static double MotionVelocity = 0.05;
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

static bool DiagnosticsEnabled = false;
static bool DualMotionEnabled = false;
static bool AllMotionEnabled = false;
static bool JointVectorMotionEnabled = false;
static bool RobotModelProbeEnabled = false;
static bool RobotPoseProbeEnabled = false;
static bool KinematicsDryRunEnabled = false;
static bool CartesianVectorMotionEnabled = false;
static bool PositionOnlyIkEnabled = false;
static bool CompactSegmentedExecutionEnabled = false;
static bool AppendSegmentedExecutionEnabled = false;
static bool TrajectorySegmentedExecutionEnabled = false;
static bool EndpointOnlyMotionEnabled = false;
static bool SegmentGoalMotionEnabled = false;
static bool CartesianVectorMotionConfirmed = false;
static double ReturnWarnToleranceUserUnits = 0.00025;
static double ReturnFailToleranceUserUnits = 0.00100;
static JointVector RequestedJointVector{};
static CartesianVector RequestedCartesianVector{};

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

JointVector makeAxis5And6Vector(double axis5Value, double axis6Value)
{
    JointVector values{};
    values[Axis5Index] = axis5Value;
    values[Axis6Index] = axis6Value;
    return values;
}

static constexpr double Pi = 3.141592653589793238462643383279502884;
static constexpr double RevolutionsToRadians = 2.0 * Pi;
static constexpr int MaxCartesianSegments = 128;

// OpenRAVE/RapidRobot Racer3 geometry extracted from racer3.kinbody.xml
// and racer3.robot.xml. Units are meters and radians.
struct FkVec3
{
    double x;
    double y;
    double z;
};

struct FkMat4
{
    double m[4][4];
};

const std::array<FkVec3, Racer3BasicMotion::AxisCount> OpenRaveJointAnchors = {{
    {0.0,  0.0, 0.0},
    {0.05, 0.0, 0.365},
    {0.05, 0.0, 0.635},
    {0.0,  0.0, 0.676},
    {0.0,  0.0, 0.94094},
    {0.0,  0.0, 1.01194}
}};

const std::array<FkVec3, Racer3BasicMotion::AxisCount> OpenRaveJointAxes = {{
    {0.0,  0.0, -1.0},
    {0.0,  1.0,  0.0},
    {0.0, -1.0,  0.0},
    {0.0,  0.0, -1.0},
    {0.0,  1.0,  0.0},
    {0.0,  0.0, -1.0}
}};

const JointVector RapidRobotAbsoluteSingleTurnHomeRadians = {
    1.6844493899891712e-06,
    0.0,
    -1.5707960000000001,
    0.0,
    1.5707960000000001,
    0.0
};

static constexpr FkVec3 OpenRaveToolPointAtZero = {0.0, 0.0, 1.012};

FkVec3 addVec(const FkVec3& a, const FkVec3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

FkVec3 subtractVec(const FkVec3& a, const FkVec3& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

FkVec3 scaleVec(const FkVec3& value, double scale)
{
    return {value.x * scale, value.y * scale, value.z * scale};
}

double dotVec(const FkVec3& a, const FkVec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

FkVec3 crossVec(const FkVec3& a, const FkVec3& b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

FkVec3 normalizeVec(const FkVec3& value)
{
    const double norm = std::sqrt(dotVec(value, value));
    if (norm <= 1e-12)
    {
        throw std::runtime_error("Cannot normalize a zero-length kinematic axis.");
    }

    return scaleVec(value, 1.0 / norm);
}

FkVec3 rotateVecAboutOrigin(const FkVec3& axisInput, double theta, const FkVec3& value)
{
    const FkVec3 axis = normalizeVec(axisInput);
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    const FkVec3 term1 = scaleVec(value, c);
    const FkVec3 term2 = scaleVec(crossVec(axis, value), s);
    const FkVec3 term3 = scaleVec(axis, dotVec(axis, value) * (1.0 - c));

    return addVec(addVec(term1, term2), term3);
}

FkMat4 identityMat4()
{
    FkMat4 result{};

    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            result.m[row][col] = (row == col) ? 1.0 : 0.0;
        }
    }

    return result;
}

FkMat4 multiplyMat4(const FkMat4& a, const FkMat4& b)
{
    FkMat4 result{};

    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            double value = 0.0;
            for (int k = 0; k < 4; ++k)
            {
                value += a.m[row][k] * b.m[k][col];
            }
            result.m[row][col] = value;
        }
    }

    return result;
}

FkMat4 revoluteTransformAboutLine(const FkVec3& axisInput, const FkVec3& anchor, double theta)
{
    const FkVec3 axis = normalizeVec(axisInput);
    const double x = axis.x;
    const double y = axis.y;
    const double z = axis.z;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double v = 1.0 - c;

    FkMat4 result = identityMat4();

    result.m[0][0] = x * x * v + c;
    result.m[0][1] = x * y * v - z * s;
    result.m[0][2] = x * z * v + y * s;

    result.m[1][0] = y * x * v + z * s;
    result.m[1][1] = y * y * v + c;
    result.m[1][2] = y * z * v - x * s;

    result.m[2][0] = z * x * v - y * s;
    result.m[2][1] = z * y * v + x * s;
    result.m[2][2] = z * z * v + c;

    const FkVec3 rotatedAnchor = rotateVecAboutOrigin(axis, theta, anchor);
    const FkVec3 translation = subtractVec(anchor, rotatedAnchor);

    result.m[0][3] = translation.x;
    result.m[1][3] = translation.y;
    result.m[2][3] = translation.z;

    return result;
}

FkMat4 zeroToolTransform()
{
    FkMat4 result = identityMat4();
    result.m[0][3] = OpenRaveToolPointAtZero.x;
    result.m[1][3] = OpenRaveToolPointAtZero.y;
    result.m[2][3] = OpenRaveToolPointAtZero.z;
    return result;
}

FkMat4 openRaveRacer3ForwardKinematics(const JointVector& jointRadians)
{
    FkMat4 result = identityMat4();

    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        result = multiplyMat4(
            result,
            revoluteTransformAboutLine(
                OpenRaveJointAxes[index],
                OpenRaveJointAnchors[index],
                jointRadians[index]));
    }

    result = multiplyMat4(result, zeroToolTransform());
    return result;
}

FkVec3 positionFromTransform(const FkMat4& transform)
{
    return {transform.m[0][3], transform.m[1][3], transform.m[2][3]};
}

FkVec3 rpyFromTransform(const FkMat4& transform)
{
    const double roll = std::atan2(transform.m[2][1], transform.m[2][2]);
    const double pitch = std::atan2(
        -transform.m[2][0],
        std::sqrt(transform.m[2][1] * transform.m[2][1] + transform.m[2][2] * transform.m[2][2]));
    const double yaw = std::atan2(transform.m[1][0], transform.m[0][0]);

    return {roll, pitch, yaw};
}

void printFkVec3(const char* label, const FkVec3& value, const char* units)
{
    std::cout << "  " << label
              << " X=" << std::fixed << std::setprecision(9) << value.x
              << " Y=" << value.y
              << " Z=" << value.z
              << " " << units << "\n";
}

void printJointRadians(const char* label, const JointVector& values)
{
    std::cout << label << ": ";
    std::cout << std::fixed << std::setprecision(9);

    for (double value : values)
    {
        std::cout << std::setw(14) << value << ' ';
    }

    std::cout << '\n';
}

void printJointDegreesFromRadians(const char* label, const JointVector& values)
{
    std::cout << label << ": ";
    std::cout << std::fixed << std::setprecision(6);

    for (double value : values)
    {
        std::cout << std::setw(11) << (value * 180.0 / Pi) << ' ';
    }

    std::cout << '\n';
}

void printCartesianVector(const char* label, const CartesianVector& values)
{
    std::cout << label
              << " [dx,dy,dz meters, droll,dpitch,dyaw radians]: ";
    std::cout << std::fixed << std::setprecision(9);

    for (double value : values)
    {
        std::cout << std::setw(14) << value << ' ';
    }

    std::cout << '\n';
}

void printOpenRaveFkReport(const char* label, const JointVector& jointRadians)
{
    const FkMat4 transform = openRaveRacer3ForwardKinematics(jointRadians);
    const FkVec3 position = positionFromTransform(transform);
    const FkVec3 rpy = rpyFromTransform(transform);

    std::cout << "\n" << label << "\n";
    printJointRadians("  Joint radians", jointRadians);
    printJointDegreesFromRadians("  Joint degrees", jointRadians);
    printFkVec3("  TCP position", position, "meters");
    printFkVec3("  TCP RPY", rpy, "radians");
    std::cout << "  TCP RPY degrees"
              << " R=" << std::fixed << std::setprecision(6) << (rpy.x * 180.0 / Pi)
              << " P=" << (rpy.y * 180.0 / Pi)
              << " Y=" << (rpy.z * 180.0 / Pi)
              << "\n";
}


double degreesToRadians(double degrees)
{
    return degrees * Pi / 180.0;
}

double wrapToPi(double value)
{
    while (value > Pi)
    {
        value -= 2.0 * Pi;
    }

    while (value < -Pi)
    {
        value += 2.0 * Pi;
    }

    return value;
}

const JointVector OpenRaveJointMinRadians = {
    degreesToRadians(-150.0),
    degreesToRadians(-95.0),
    degreesToRadians(-155.0),
    degreesToRadians(-200.0),
    degreesToRadians(-125.0),
    degreesToRadians(-540.0)
};

const JointVector OpenRaveJointMaxRadians = {
    degreesToRadians(150.0),
    degreesToRadians(135.0),
    degreesToRadians(90.0),
    degreesToRadians(200.0),
    degreesToRadians(125.0),
    degreesToRadians(540.0)
};

struct IkDryRunResult
{
    bool converged = false;
    bool hitJointLimit = false;
    int iterations = 0;
    double residualNorm = 0.0;
    double maxResidualComponent = 0.0;
    JointVector solutionRadians{};
    JointVector deltaRadians{};
    CartesianVector residual{};
};

// Dry-run validation gates for promoting an IK result toward future motion testing.
// These are intentionally conservative. The code still does not command motion.
static constexpr double CandidateResidualNormAccept = 4e-3;
static constexpr double CandidateMaxResidualComponentAccept = 4e-3;
static constexpr double CandidateMaxJointDeltaDegreesAccept = 90.0;

// Endpoint-only point motion is a full joint-space move to one final XYZ target,
// not a tiny per-segment Cartesian step. Keep a hard gate, but make it separate
// from the segmented Cartesian per-step gate.
static constexpr double EndpointMaxJointDeltaDegreesAccept = 180.0;
static constexpr double EndpointWarnJointDeltaDegrees = 90.0;
static constexpr int EndpointPvtWaypointCount = 96;

CartesianVector poseVectorFromJoints(const JointVector& jointRadians)
{
    const FkMat4 transform = openRaveRacer3ForwardKinematics(jointRadians);
    const FkVec3 position = positionFromTransform(transform);
    const FkVec3 rpy = rpyFromTransform(transform);

    return {
        position.x,
        position.y,
        position.z,
        rpy.x,
        rpy.y,
        rpy.z
    };
}

CartesianVector subtractPoseVectorWrapped(const CartesianVector& target, const CartesianVector& current)
{
    return {
        target[0] - current[0],
        target[1] - current[1],
        target[2] - current[2],
        wrapToPi(target[3] - current[3]),
        wrapToPi(target[4] - current[4]),
        wrapToPi(target[5] - current[5])
    };
}

double residualNorm(const CartesianVector& residual)
{
    double sum = 0.0;

    for (double value : residual)
    {
        sum += value * value;
    }

    return std::sqrt(sum);
}

double maxAbsResidualComponent(const CartesianVector& residual)
{
    double result = 0.0;

    for (double value : residual)
    {
        result = std::max(result, std::fabs(value));
    }

    return result;
}

bool residualComponentParticipatesInIk(int componentIndex)
{
    return !PositionOnlyIkEnabled || componentIndex < 3;
}

double residualNormForIkMode(const CartesianVector& residual)
{
    double sum = 0.0;

    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        if (!residualComponentParticipatesInIk(index))
        {
            continue;
        }

        sum += residual[index] * residual[index];
    }

    return std::sqrt(sum);
}

double maxAbsResidualComponentForIkMode(const CartesianVector& residual)
{
    double result = 0.0;

    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        if (!residualComponentParticipatesInIk(index))
        {
            continue;
        }

        result = std::max(result, std::fabs(residual[index]));
    }

    return result;
}

const char* ikResidualModeName()
{
    return PositionOnlyIkEnabled ? "POSITION_ONLY_XYZ" : "FULL_POSE_XYZ_RPY";
}

const char* compactSegmentedExecutionName()
{
    return CompactSegmentedExecutionEnabled ? "COMPACT" : "VERBOSE";
}

const char* segmentedMotionExecutionModeName()
{
    if (SegmentGoalMotionEnabled)
    {
        return "SEGMENT_GOAL_PVT_EXPERIMENTAL";
    }

    if (EndpointOnlyMotionEnabled)
    {
        return "ENDPOINT_ONLY_PVT_EXPERIMENTAL";
    }

    if (TrajectorySegmentedExecutionEnabled)
    {
        return "PVT_TRAJECTORY_EXPERIMENTAL";
    }

    if (AppendSegmentedExecutionEnabled)
    {
        return "APPEND_QUEUED_EXPERIMENTAL";
    }

    return "STEP_WAIT";
}

void printCartesianResidual(const char* label, const CartesianVector& residual)
{
    std::cout << label
              << " [dx,dy,dz meters, droll,dpitch,dyaw radians]: ";
    std::cout << std::fixed << std::setprecision(9);

    for (double value : residual)
    {
        std::cout << std::setw(14) << value << ' ';
    }

    std::cout << '\n';
}

void printJointUserUnitsFromRadians(const char* label, const JointVector& values)
{
    std::cout << label << ": ";
    std::cout << std::fixed << std::setprecision(9);

    for (double value : values)
    {
        std::cout << std::setw(14) << (value / RevolutionsToRadians) << ' ';
    }

    std::cout << '\n';
}

void printJointLimitReport(const JointVector& jointRadians)
{
    std::cout << "  Joint limit check using rapidrobot test-driver limits:\n";

    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        const bool below = jointRadians[index] < OpenRaveJointMinRadians[index];
        const bool above = jointRadians[index] > OpenRaveJointMaxRadians[index];
        const char* status = (below || above) ? "OUTSIDE" : "inside";

        std::cout << "    J" << (index + 1)
                  << " q=" << std::fixed << std::setprecision(6) << (jointRadians[index] * 180.0 / Pi)
                  << " deg, limit=["
                  << (OpenRaveJointMinRadians[index] * 180.0 / Pi)
                  << ", "
                  << (OpenRaveJointMaxRadians[index] * 180.0 / Pi)
                  << "] deg => "
                  << status
                  << "\n";
    }
}

bool isOutsideJointLimits(const JointVector& jointRadians)
{
    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        if (jointRadians[index] < OpenRaveJointMinRadians[index] ||
            jointRadians[index] > OpenRaveJointMaxRadians[index])
        {
            return true;
        }
    }

    return false;
}

void clampToJointLimits(JointVector& jointRadians, bool& hitLimit)
{
    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        if (jointRadians[index] < OpenRaveJointMinRadians[index])
        {
            jointRadians[index] = OpenRaveJointMinRadians[index];
            hitLimit = true;
        }

        if (jointRadians[index] > OpenRaveJointMaxRadians[index])
        {
            jointRadians[index] = OpenRaveJointMaxRadians[index];
            hitLimit = true;
        }
    }
}

bool solveLinearSystem6x6(
    std::array<std::array<double, Racer3BasicMotion::AxisCount>, Racer3BasicMotion::AxisCount> matrix,
    std::array<double, Racer3BasicMotion::AxisCount> rhs,
    std::array<double, Racer3BasicMotion::AxisCount>& solution)
{
    constexpr int n = Racer3BasicMotion::AxisCount;

    for (int pivot = 0; pivot < n; ++pivot)
    {
        int bestRow = pivot;
        double bestAbs = std::fabs(matrix[pivot][pivot]);

        for (int row = pivot + 1; row < n; ++row)
        {
            const double candidate = std::fabs(matrix[row][pivot]);
            if (candidate > bestAbs)
            {
                bestAbs = candidate;
                bestRow = row;
            }
        }

        if (bestAbs < 1e-14)
        {
            return false;
        }

        if (bestRow != pivot)
        {
            std::swap(matrix[pivot], matrix[bestRow]);
            std::swap(rhs[pivot], rhs[bestRow]);
        }

        const double diagonal = matrix[pivot][pivot];
        for (int col = pivot; col < n; ++col)
        {
            matrix[pivot][col] /= diagonal;
        }
        rhs[pivot] /= diagonal;

        for (int row = 0; row < n; ++row)
        {
            if (row == pivot)
            {
                continue;
            }

            const double factor = matrix[row][pivot];
            for (int col = pivot; col < n; ++col)
            {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    for (int index = 0; index < n; ++index)
    {
        solution[index] = rhs[index];
    }

    return true;
}

IkDryRunResult solveNumericalIkDampedLeastSquares(
    const JointVector& seedRadians,
    const CartesianVector& targetPoseVector)
{
    static constexpr int MaxIterations = 160;
    static constexpr double FiniteDifferenceStepRadians = 1e-5;
    static constexpr double Damping = 0.025;
    static constexpr double MaxJointStepRadians = 0.050;
    static constexpr double ConvergedResidualNorm = 1e-5;
    static constexpr double ConvergedMaxComponent = 1e-5;

    IkDryRunResult result{};
    JointVector q = seedRadians;

    clampToJointLimits(q, result.hitJointLimit);

    for (int iteration = 0; iteration < MaxIterations; ++iteration)
    {
        const CartesianVector currentPose = poseVectorFromJoints(q);
        const CartesianVector error = subtractPoseVectorWrapped(targetPoseVector, currentPose);

        result.iterations = iteration;
        result.residual = error;
        result.residualNorm = residualNormForIkMode(error);
        result.maxResidualComponent = maxAbsResidualComponentForIkMode(error);

        if (result.residualNorm < ConvergedResidualNorm &&
            result.maxResidualComponent < ConvergedMaxComponent)
        {
            result.converged = true;
            break;
        }

        std::array<std::array<double, Racer3BasicMotion::AxisCount>, Racer3BasicMotion::AxisCount> jacobian{};

        for (int joint = 0; joint < Racer3BasicMotion::AxisCount; ++joint)
        {
            JointVector qPerturbed = q;
            qPerturbed[joint] += FiniteDifferenceStepRadians;

            const CartesianVector perturbedPose = poseVectorFromJoints(qPerturbed);
            const CartesianVector difference = subtractPoseVectorWrapped(perturbedPose, currentPose);

            for (int row = 0; row < Racer3BasicMotion::AxisCount; ++row)
            {
                jacobian[row][joint] = difference[row] / FiniteDifferenceStepRadians;
            }
        }

        std::array<std::array<double, Racer3BasicMotion::AxisCount>, Racer3BasicMotion::AxisCount> normal{};
        std::array<double, Racer3BasicMotion::AxisCount> rhs{};

        for (int row = 0; row < Racer3BasicMotion::AxisCount; ++row)
        {
            for (int col = 0; col < Racer3BasicMotion::AxisCount; ++col)
            {
                double value = 0.0;
                for (int k = 0; k < Racer3BasicMotion::AxisCount; ++k)
                {
                    if (!residualComponentParticipatesInIk(k))
                    {
                        continue;
                    }

                    value += jacobian[k][row] * jacobian[k][col];
                }

                if (row == col)
                {
                    value += Damping * Damping;
                }

                normal[row][col] = value;
            }

            double rhsValue = 0.0;
            for (int k = 0; k < Racer3BasicMotion::AxisCount; ++k)
            {
                if (!residualComponentParticipatesInIk(k))
                {
                    continue;
                }

                rhsValue += jacobian[k][row] * error[k];
            }
            rhs[row] = rhsValue;
        }

        std::array<double, Racer3BasicMotion::AxisCount> delta{};

        if (!solveLinearSystem6x6(normal, rhs, delta))
        {
            break;
        }

        double maxStep = 0.0;
        for (double value : delta)
        {
            maxStep = std::max(maxStep, std::fabs(value));
        }

        if (maxStep > MaxJointStepRadians)
        {
            const double scale = MaxJointStepRadians / maxStep;
            for (double& value : delta)
            {
                value *= scale;
            }
        }

        for (int joint = 0; joint < Racer3BasicMotion::AxisCount; ++joint)
        {
            q[joint] += delta[joint];
        }

        clampToJointLimits(q, result.hitJointLimit);
    }

    const CartesianVector finalPose = poseVectorFromJoints(q);
    result.residual = subtractPoseVectorWrapped(targetPoseVector, finalPose);
    result.residualNorm = residualNormForIkMode(result.residual);
    result.maxResidualComponent = maxAbsResidualComponentForIkMode(result.residual);
    result.solutionRadians = q;

    for (int joint = 0; joint < Racer3BasicMotion::AxisCount; ++joint)
    {
        result.deltaRadians[joint] = q[joint] - seedRadians[joint];
    }

    if (result.residualNorm < 1e-5 && result.maxResidualComponent < 1e-5)
    {
        result.converged = true;
    }

    result.hitJointLimit = result.hitJointLimit || isOutsideJointLimits(result.solutionRadians);

    return result;
}

void printIkDryRunReport(
    const char* label,
    const JointVector& seedRadians,
    const CartesianVector& targetPoseVector)
{
    std::cout << "\n" << label << "\n";

    const IkDryRunResult result = solveNumericalIkDampedLeastSquares(
        seedRadians,
        targetPoseVector);

    std::cout << "  Solver: damped least-squares numerical IK, finite-difference Jacobian.\n";
    std::cout << "  IK residual mode: " << ikResidualModeName() << "\n";
    if (PositionOnlyIkEnabled)
    {
        std::cout << "  Position-only IK ignores roll/pitch/yaw in solve and validation, but still prints full pose residual.\n";
    }
    std::cout << "  Converged: " << (result.converged ? "true" : "false") << "\n";
    std::cout << "  Iterations: " << result.iterations << "\n";
    std::cout << "  Residual norm: " << std::fixed << std::setprecision(9) << result.residualNorm << "\n";
    std::cout << "  Max residual component: " << result.maxResidualComponent << "\n";
    std::cout << "  Hit joint limit during solve: " << (result.hitJointLimit ? "true" : "false") << "\n";
    printCartesianResidual("  Final residual", result.residual);

    printJointRadians("  Seed joint radians", seedRadians);
    printJointDegreesFromRadians("  Seed joint degrees", seedRadians);

    printJointRadians("  Candidate joint radians", result.solutionRadians);
    printJointDegreesFromRadians("  Candidate joint degrees", result.solutionRadians);

    printJointRadians("  Candidate delta radians", result.deltaRadians);
    printJointDegreesFromRadians("  Candidate delta degrees", result.deltaRadians);
    printJointUserUnitsFromRadians("  Candidate delta user units/revolutions", result.deltaRadians);

    printJointLimitReport(result.solutionRadians);
    printOpenRaveFkReport("  FK of candidate IK solution", result.solutionRadians);

    if (result.converged && !result.hitJointLimit)
    {
        std::cout << "  IK dry-run verdict: candidate joint-vector found. Motion is still disabled in this patch.\n";
    }
    else
    {
        std::cout << "  IK dry-run verdict: no validated candidate for motion yet. Do not command this target.\n";
    }
}


struct IkSeedCandidate
{
    std::string name;
    JointVector seedRadians{};
    IkDryRunResult result{};
    double score = std::numeric_limits<double>::infinity();
};

struct IkBestCandidate
{
    bool found = false;
    std::string seedName;
    JointVector seedRadians{};
    IkDryRunResult result{};
    double score = std::numeric_limits<double>::infinity();
};

JointVector withJointOffsetDegrees(const JointVector& base, int jointIndex, double degrees)
{
    JointVector seed = base;
    seed[jointIndex] += degreesToRadians(degrees);
    return seed;
}

JointVector withBendOffsetDegrees(
    const JointVector& base,
    double joint2Degrees,
    double joint3Degrees,
    double joint5Degrees)
{
    JointVector seed = base;
    seed[1] += degreesToRadians(joint2Degrees);
    seed[2] += degreesToRadians(joint3Degrees);
    seed[4] += degreesToRadians(joint5Degrees);
    return seed;
}

std::vector<IkSeedCandidate> makeIkSeedCandidates(const JointVector& baseSeedRadians)
{
    std::vector<IkSeedCandidate> candidates;

    auto addSeed = [&](const std::string& name, JointVector seed) {
        bool hitLimit = false;
        clampToJointLimits(seed, hitLimit);
        candidates.push_back({name, seed, {}, std::numeric_limits<double>::infinity()});
    };

    addSeed("current", baseSeedRadians);
    addSeed("J1 +5 deg", withJointOffsetDegrees(baseSeedRadians, 0, 5.0));
    addSeed("J1 -5 deg", withJointOffsetDegrees(baseSeedRadians, 0, -5.0));
    addSeed("J1 +15 deg", withJointOffsetDegrees(baseSeedRadians, 0, 15.0));
    addSeed("J1 -15 deg", withJointOffsetDegrees(baseSeedRadians, 0, -15.0));

    // Wider J1 seeds are important near the software-zero vertical pose.
    // At that pose the TCP is nearly on the base rotation axis, so pure +/-Y
    // targets can require rotating the arm into a different azimuth before
    // bending J2/J3/J5 creates useful lateral reach.
    addSeed("J1 +45 deg", withJointOffsetDegrees(baseSeedRadians, 0, 45.0));
    addSeed("J1 -45 deg", withJointOffsetDegrees(baseSeedRadians, 0, -45.0));
    addSeed("J1 +90 deg", withJointOffsetDegrees(baseSeedRadians, 0, 90.0));
    addSeed("J1 -90 deg", withJointOffsetDegrees(baseSeedRadians, 0, -90.0));
    addSeed("J1 +135 deg", withJointOffsetDegrees(baseSeedRadians, 0, 135.0));
    addSeed("J1 -135 deg", withJointOffsetDegrees(baseSeedRadians, 0, -135.0));

    addSeed("small positive bend", withBendOffsetDegrees(baseSeedRadians, 2.0, 2.0, -0.5));
    addSeed("small negative bend", withBendOffsetDegrees(baseSeedRadians, -2.0, -2.0, -0.5));
    addSeed("larger positive bend", withBendOffsetDegrees(baseSeedRadians, 8.0, 8.0, -1.0));
    addSeed("larger negative bend", withBendOffsetDegrees(baseSeedRadians, -8.0, -8.0, -1.0));

    addSeed("J1 +5 deg + positive bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, 5.0), 4.0, 4.0, -0.5));
    addSeed("J1 -5 deg + positive bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, -5.0), 4.0, 4.0, -0.5));
    addSeed("J1 +5 deg + negative bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, 5.0), -4.0, -4.0, -0.5));
    addSeed("J1 -5 deg + negative bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, -5.0), -4.0, -4.0, -0.5));

    addSeed("J1 +45 deg + positive bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, 45.0), 4.0, 4.0, -0.5));
    addSeed("J1 -45 deg + positive bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, -45.0), 4.0, 4.0, -0.5));
    addSeed("J1 +45 deg + negative bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, 45.0), -4.0, -4.0, -0.5));
    addSeed("J1 -45 deg + negative bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, -45.0), -4.0, -4.0, -0.5));

    addSeed("J1 +90 deg + positive bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, 90.0), 4.0, 4.0, -0.5));
    addSeed("J1 -90 deg + positive bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, -90.0), 4.0, 4.0, -0.5));
    addSeed("J1 +90 deg + negative bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, 90.0), -4.0, -4.0, -0.5));
    addSeed("J1 -90 deg + negative bend", withBendOffsetDegrees(withJointOffsetDegrees(baseSeedRadians, 0, -90.0), -4.0, -4.0, -0.5));

    return candidates;
}

double candidateJointDeltaNorm(const JointVector& deltaRadians)
{
    double sum = 0.0;

    for (double value : deltaRadians)
    {
        sum += value * value;
    }

    return std::sqrt(sum);
}

double maxAbsJointDeltaDegrees(const JointVector& deltaRadians)
{
    double result = 0.0;

    for (double value : deltaRadians)
    {
        result = std::max(result, std::fabs(value * 180.0 / Pi));
    }

    return result;
}

bool residualAcceptedForDryRunCandidate(const IkDryRunResult& result)
{
    return result.residualNorm <= CandidateResidualNormAccept &&
           result.maxResidualComponent <= CandidateMaxResidualComponentAccept;
}

bool jointDeltaAcceptedForDryRunCandidate(const IkDryRunResult& result)
{
    return maxAbsJointDeltaDegrees(result.deltaRadians) <= CandidateMaxJointDeltaDegreesAccept;
}

bool acceptedForFutureMotionDryRunCandidate(const IkDryRunResult& result)
{
    return residualAcceptedForDryRunCandidate(result) &&
           jointDeltaAcceptedForDryRunCandidate(result) &&
           !result.hitJointLimit;
}

double scoreIkCandidate(const IkDryRunResult& result)
{
    const double jointDeltaNorm = candidateJointDeltaNorm(result.deltaRadians);
    double score =
        result.residualNorm +
        0.25 * result.maxResidualComponent +
        0.001 * jointDeltaNorm;

    if (!result.converged)
    {
        score += 10.0;
    }

    if (result.hitJointLimit)
    {
        score += 100.0;
    }

    return score;
}

void printCompactIkCandidateLine(const IkSeedCandidate& candidate)
{
    const double maxDeltaDegrees = maxAbsJointDeltaDegrees(candidate.result.deltaRadians);

    std::cout << "  "
              << std::left << std::setw(28) << candidate.name
              << " conv=" << std::setw(5) << (candidate.result.converged ? "true" : "false")
              << " limit=" << std::setw(5) << (candidate.result.hitJointLimit ? "true" : "false")
              << " resOK=" << std::setw(5) << (residualAcceptedForDryRunCandidate(candidate.result) ? "true" : "false")
              << " dOK=" << std::setw(5) << (jointDeltaAcceptedForDryRunCandidate(candidate.result) ? "true" : "false")
              << " ikMode=" << ikResidualModeName()
              << " iter=" << std::right << std::setw(3) << candidate.result.iterations
              << " norm=" << std::fixed << std::setprecision(9) << candidate.result.residualNorm
              << " max=" << candidate.result.maxResidualComponent
              << " maxDdeg=" << maxDeltaDegrees
              << " score=" << candidate.score
              << "\n";
}

IkBestCandidate solveBestMultiSeedIkCandidate(
    const JointVector& baseSeedRadians,
    const CartesianVector& targetPoseVector)
{
    std::vector<IkSeedCandidate> candidates = makeIkSeedCandidates(baseSeedRadians);

    IkBestCandidate best{};

    for (IkSeedCandidate& candidate : candidates)
    {
        candidate.result = solveNumericalIkDampedLeastSquares(
            candidate.seedRadians,
            targetPoseVector);

        candidate.score = scoreIkCandidate(candidate.result);

        if (!best.found || candidate.score < best.score)
        {
            best.found = true;
            best.seedName = candidate.name;
            best.seedRadians = candidate.seedRadians;
            best.result = candidate.result;
            best.score = candidate.score;
        }
    }

    return best;
}

JointVector subtractJointVectors(const JointVector& a, const JointVector& b)
{
    JointVector result{};

    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        result[index] = a[index] - b[index];
    }

    return result;
}

JointVector radiansToUserUnits(const JointVector& radians)
{
    JointVector result{};

    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        result[index] = radians[index] / RevolutionsToRadians;
    }

    return result;
}

void printCartesianVectorMotionCandidateSummary(
    const IkBestCandidate& candidate,
    const JointVector& currentRadians,
    const CartesianVector& targetPoseVector)
{
    if (!candidate.found)
    {
        std::cout << "No IK candidate was evaluated.\n";
        return;
    }

    const JointVector commandDeltaRadians =
        subtractJointVectors(candidate.result.solutionRadians, currentRadians);
    const JointVector commandDeltaUserUnits = radiansToUserUnits(commandDeltaRadians);
    const double commandMaxDeltaDegrees = maxAbsJointDeltaDegrees(commandDeltaRadians);

    const bool residualOk = residualAcceptedForDryRunCandidate(candidate.result);
    const bool commandDeltaOk = commandMaxDeltaDegrees <= CandidateMaxJointDeltaDegreesAccept;
    const bool futureMotionCandidateOk =
        residualOk &&
        commandDeltaOk &&
        !candidate.result.hitJointLimit;

    std::cout << "\nGuarded Cartesian-vector candidate summary\n";
    std::cout << "  IK residual mode: " << ikResidualModeName() << "\n";
    if (PositionOnlyIkEnabled)
    {
        std::cout << "  Position-only IK: orientation residual is reported but does not block the candidate.\n";
    }
    std::cout << "  Best seed: " << candidate.seedName << "\n";
    std::cout << "  Solver converged flag: " << (candidate.result.converged ? "true" : "false") << "\n";
    std::cout << "  Hit joint limit: " << (candidate.result.hitJointLimit ? "true" : "false") << "\n";
    std::cout << "  Residual norm: " << std::fixed << std::setprecision(9) << candidate.result.residualNorm << "\n";
    std::cout << "  Max residual component: " << candidate.result.maxResidualComponent << "\n";
    std::cout << "  Max command joint delta from current pose: " << commandMaxDeltaDegrees << " degrees\n";
    std::cout << "  Residual gate <= " << CandidateResidualNormAccept
              << " norm and <= "
              << CandidateMaxResidualComponentAccept
              << " max component: "
              << (residualOk ? "PASS" : "FAIL")
              << "\n";
    std::cout << "  Joint-delta gate <= " << CandidateMaxJointDeltaDegreesAccept
              << " deg max command delta: "
              << (commandDeltaOk ? "PASS" : "FAIL")
              << "\n";
    std::cout << "  Future motion dry-run candidate: "
              << (futureMotionCandidateOk ? "YES" : "NO")
              << "\n";

    printCartesianResidual("  Final residual", candidate.result.residual);
    printJointDegreesFromRadians("  Current joint degrees", currentRadians);
    printJointDegreesFromRadians("  Candidate joint degrees", candidate.result.solutionRadians);
    printJointDegreesFromRadians("  Command delta degrees", commandDeltaRadians);
    printJointUserUnitsFromRadians("  Command delta user units/revolutions", commandDeltaRadians);
    printJointLimitReport(candidate.result.solutionRadians);
    printOpenRaveFkReport("  FK verification for guarded Cartesian-vector candidate", candidate.result.solutionRadians);

    if (!futureMotionCandidateOk)
    {
        std::cout << "  Guarded Cartesian-vector verdict: rejected. Motion will not be commanded.\n";

        if (residualOk && !commandDeltaOk)
        {
            std::cout << "  Note: the solver got close, but the command delta is too large for this guarded tiny-motion path.\n";
            std::cout << "  Use a staged ready/pre-bend pose or a smaller/different Cartesian delta before live testing.\n";
        }
    }
    else
    {
        std::cout << "  Guarded Cartesian-vector verdict: accepted by dry-run validation gates.\n";
    }
}


CartesianVector scaleCartesianVector(const CartesianVector& values, double scale)
{
    CartesianVector result{};

    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        result[index] = values[index] * scale;
    }

    return result;
}

CartesianVector targetPoseVectorForCartesianDelta(
    const JointVector& currentRadians,
    const CartesianVector& delta)
{
    const FkMat4 currentTransform = openRaveRacer3ForwardKinematics(currentRadians);
    const FkVec3 currentPosition = positionFromTransform(currentTransform);
    const FkVec3 currentRpy = rpyFromTransform(currentTransform);

    return {
        currentPosition.x + delta[0],
        currentPosition.y + delta[1],
        currentPosition.z + delta[2],
        currentRpy.x + delta[3],
        currentRpy.y + delta[4],
        currentRpy.z + delta[5]
    };
}

CartesianVector targetPoseVectorFromStartPoseAndDelta(
    const CartesianVector& startPoseVector,
    const CartesianVector& requestedCartesianDelta,
    double fraction)
{
    return {
        startPoseVector[0] + requestedCartesianDelta[0] * fraction,
        startPoseVector[1] + requestedCartesianDelta[1] * fraction,
        startPoseVector[2] + requestedCartesianDelta[2] * fraction,
        startPoseVector[3] + requestedCartesianDelta[3] * fraction,
        startPoseVector[4] + requestedCartesianDelta[4] * fraction,
        startPoseVector[5] + requestedCartesianDelta[5] * fraction
    };
}

IkBestCandidate solveBestCandidateForCartesianDelta(
    const JointVector& currentRadians,
    const CartesianVector& delta)
{
    return solveBestMultiSeedIkCandidate(
        currentRadians,
        targetPoseVectorForCartesianDelta(currentRadians, delta));
}

struct CartesianSegmentCandidate
{
    int segmentNumber = 0;
    CartesianVector requestedCartesianDelta{};
    JointVector startRadians{};
    IkBestCandidate candidate{};
    JointVector commandDeltaRadians{};
    JointVector commandDeltaUserUnits{};
    double maxCommandDeltaDegrees = 0.0;
    bool accepted = false;
};

struct CartesianSegmentPlan
{
    bool accepted = false;
    int segmentCount = 0;
    std::vector<CartesianSegmentCandidate> segments;
    std::string rejectionReason;
};

bool guardedCandidateAcceptedFromCurrent(
    const IkBestCandidate& candidate,
    const JointVector& currentRadians,
    double* maxCommandDeltaDegrees = nullptr)
{
    if (!candidate.found)
    {
        if (maxCommandDeltaDegrees)
        {
            *maxCommandDeltaDegrees = 0.0;
        }

        return false;
    }

    const JointVector commandDeltaRadians =
        subtractJointVectors(candidate.result.solutionRadians, currentRadians);

    const double maxDelta = maxAbsJointDeltaDegrees(commandDeltaRadians);

    if (maxCommandDeltaDegrees)
    {
        *maxCommandDeltaDegrees = maxDelta;
    }

    return residualAcceptedForDryRunCandidate(candidate.result) &&
           maxDelta <= CandidateMaxJointDeltaDegreesAccept &&
           !candidate.result.hitJointLimit;
}

CartesianSegmentCandidate makeCartesianSegmentCandidateForTargetPose(
    int segmentNumber,
    const JointVector& currentRadians,
    const CartesianVector& reportedSegmentDelta,
    const CartesianVector& absoluteTargetPoseVector)
{
    CartesianSegmentCandidate segment{};
    segment.segmentNumber = segmentNumber;
    segment.requestedCartesianDelta = reportedSegmentDelta;
    segment.startRadians = currentRadians;

    // Important: solve this segment against the absolute waypoint on the original
    // start-to-final line. The previous implementation solved each segment as
    // current FK + small delta, which allowed a few millimeters of local residual
    // to accumulate into centimeters of final target miss.
    segment.candidate =
        solveBestMultiSeedIkCandidate(currentRadians, absoluteTargetPoseVector);

    if (segment.candidate.found)
    {
        segment.commandDeltaRadians =
            subtractJointVectors(segment.candidate.result.solutionRadians, currentRadians);
        segment.commandDeltaUserUnits = radiansToUserUnits(segment.commandDeltaRadians);
        segment.accepted = guardedCandidateAcceptedFromCurrent(
            segment.candidate,
            currentRadians,
            &segment.maxCommandDeltaDegrees);
    }

    return segment;
}

CartesianSegmentPlan buildSegmentedCartesianPlan(
    const JointVector& startRadians,
    const CartesianVector& requestedCartesianDelta)
{
    CartesianSegmentPlan bestRejected{};
    bestRejected.rejectionReason = "No segmented Cartesian plan was evaluated.";

    const CartesianVector startPoseVector = poseVectorFromJoints(startRadians);
    const CartesianVector finalTargetPoseVector =
        targetPoseVectorFromStartPoseAndDelta(
            startPoseVector,
            requestedCartesianDelta,
            1.0);

    for (int segmentCount = 1; segmentCount <= MaxCartesianSegments; ++segmentCount)
    {
        CartesianSegmentPlan plan{};
        plan.segmentCount = segmentCount;

        const CartesianVector reportedSegmentDelta =
            scaleCartesianVector(requestedCartesianDelta, 1.0 / static_cast<double>(segmentCount));

        JointVector currentRadians = startRadians;
        bool allSegmentsAccepted = true;

        for (int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
        {
            const double waypointFraction =
                static_cast<double>(segmentIndex + 1) /
                static_cast<double>(segmentCount);
            const CartesianVector waypointTargetPoseVector =
                targetPoseVectorFromStartPoseAndDelta(
                    startPoseVector,
                    requestedCartesianDelta,
                    waypointFraction);

            CartesianSegmentCandidate segment =
                makeCartesianSegmentCandidateForTargetPose(
                    segmentIndex + 1,
                    currentRadians,
                    reportedSegmentDelta,
                    waypointTargetPoseVector);

            if (!segment.accepted)
            {
                allSegmentsAccepted = false;

                if (!segment.candidate.found)
                {
                    plan.rejectionReason =
                        "Segment " + std::to_string(segmentIndex + 1) +
                        " did not produce an IK candidate.";
                }
                else if (segment.candidate.result.hitJointLimit)
                {
                    plan.rejectionReason =
                        "Segment " + std::to_string(segmentIndex + 1) +
                        " hit a joint limit.";
                }
                else if (!residualAcceptedForDryRunCandidate(segment.candidate.result))
                {
                    plan.rejectionReason =
                        "Segment " + std::to_string(segmentIndex + 1) +
                        " failed the absolute-waypoint residual gate.";
                }
                else
                {
                    plan.rejectionReason =
                        "Segment " + std::to_string(segmentIndex + 1) +
                        " failed the joint-delta gate.";
                }

                plan.segments.push_back(segment);
                break;
            }

            currentRadians = segment.candidate.result.solutionRadians;
            plan.segments.push_back(segment);
        }

        if (allSegmentsAccepted)
        {
            const CartesianVector finalPoseVector = poseVectorFromJoints(currentRadians);
            const CartesianVector finalResidual =
                subtractPoseVectorWrapped(finalTargetPoseVector, finalPoseVector);
            const double finalResidualNorm = residualNormForIkMode(finalResidual);
            const double finalMaxResidualComponent =
                maxAbsResidualComponentForIkMode(finalResidual);

            if (finalResidualNorm <= CandidateResidualNormAccept &&
                finalMaxResidualComponent <= CandidateMaxResidualComponentAccept)
            {
                plan.accepted = true;
                return plan;
            }

            plan.rejectionReason =
                "All absolute-waypoint segments passed, but final global target verification failed: residualNorm=" +
                std::to_string(finalResidualNorm) +
                ", maxResidualComponent=" +
                std::to_string(finalMaxResidualComponent) +
                ".";

            allSegmentsAccepted = false;
        }

        bestRejected = plan;
    }

    if (bestRejected.rejectionReason.empty())
    {
        bestRejected.rejectionReason =
            "No segmented Cartesian plan passed all absolute-waypoint validation gates.";
    }

    return bestRejected;
}

void printCartesianSegmentPlanSummary(const CartesianSegmentPlan& plan)
{
    std::cout << "\nSegmented Cartesian-vector plan summary\n";
    std::cout << "  IK residual mode: " << ikResidualModeName() << "\n";
    if (PositionOnlyIkEnabled)
    {
        std::cout << "  Position-only IK is active: gates check XYZ residual only; RPY residual is informational.\n";
    }
    std::cout << "  Accepted: " << (plan.accepted ? "YES" : "NO") << "\n";
    std::cout << "  Segment count: " << plan.segmentCount << "\n";
    std::cout << "  Waypoint target mode: ABSOLUTE_FROM_ORIGINAL_START_POSE\n";


    if (!plan.accepted)
    {
        std::cout << "  Rejection reason: " << plan.rejectionReason << "\n";
    }

    for (const CartesianSegmentCandidate& segment : plan.segments)
    {
        std::cout << "\n  Segment " << segment.segmentNumber << " / " << plan.segmentCount << "\n";
        printCartesianVector("    Requested segment delta", segment.requestedCartesianDelta);

        if (!segment.candidate.found)
        {
            std::cout << "    No IK candidate found.\n";
            continue;
        }

        std::cout << "    Best seed: " << segment.candidate.seedName << "\n";
        std::cout << "    Solver converged flag: "
                  << (segment.candidate.result.converged ? "true" : "false") << "\n";
        std::cout << "    Hit joint limit: "
                  << (segment.candidate.result.hitJointLimit ? "true" : "false") << "\n";
        std::cout << "    Gated residual norm: "
                  << std::fixed << std::setprecision(9)
                  << segment.candidate.result.residualNorm << "\n";
        std::cout << "    Gated max residual component: "
                  << segment.candidate.result.maxResidualComponent << "\n";
        if (PositionOnlyIkEnabled)
        {
            std::cout << "    Full pose residual norm including RPY: "
                      << residualNorm(segment.candidate.result.residual) << "\n";
            std::cout << "    Full pose max residual component including RPY: "
                      << maxAbsResidualComponent(segment.candidate.result.residual) << "\n";
        }
        std::cout << "    Max command joint delta from segment start: "
                  << segment.maxCommandDeltaDegrees << " degrees\n";
        std::cout << "    Residual gate: "
                  << (residualAcceptedForDryRunCandidate(segment.candidate.result) ? "PASS" : "FAIL")
                  << "\n";
        std::cout << "    Joint-delta gate: "
                  << (segment.maxCommandDeltaDegrees <= CandidateMaxJointDeltaDegreesAccept ? "PASS" : "FAIL")
                  << "\n";
        std::cout << "    Segment accepted: "
                  << (segment.accepted ? "YES" : "NO")
                  << "\n";
        printJointDegreesFromRadians("    Command delta degrees", segment.commandDeltaRadians);
        std::cout << "    Command delta user units/revolutions: ";
        printJointVector(segment.commandDeltaUserUnits);
    }
}

std::vector<JointVector> makeOutboundSequenceFromSegmentPlan(
    const CartesianSegmentPlan& plan)
{
    std::vector<JointVector> sequence;

    for (const CartesianSegmentCandidate& segment : plan.segments)
    {
        sequence.push_back(segment.commandDeltaUserUnits);
    }

    return sequence;
}

std::vector<JointVector> makeReturnSequenceFromSegmentPlan(
    const CartesianSegmentPlan& plan)
{
    std::vector<JointVector> sequence;

    for (auto it = plan.segments.rbegin(); it != plan.segments.rend(); ++it)
    {
        JointVector negativeReturnStep{};

        for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
        {
            negativeReturnStep[index] = -it->commandDeltaUserUnits[index];
        }

        sequence.push_back(negativeReturnStep);
    }

    return sequence;
}

std::vector<JointVector> makeOutAndBackSequenceFromSegmentPlan(
    const CartesianSegmentPlan& plan)
{
    std::vector<JointVector> sequence = makeOutboundSequenceFromSegmentPlan(plan);
    std::vector<JointVector> returnSequence = makeReturnSequenceFromSegmentPlan(plan);

    sequence.insert(sequence.end(), returnSequence.begin(), returnSequence.end());

    return sequence;
}


void printMultiSeedIkDryRunReport(
    const char* label,
    const JointVector& baseSeedRadians,
    const CartesianVector& targetPoseVector)
{
    std::cout << "\n" << label << "\n";
    std::cout << "  Multi-seed numerical IK search with wide J1 azimuth seeds. Motion is disabled.\n";

    std::vector<IkSeedCandidate> candidates = makeIkSeedCandidates(baseSeedRadians);

    int bestIndex = -1;
    double bestScore = std::numeric_limits<double>::infinity();

    for (size_t index = 0; index < candidates.size(); ++index)
    {
        candidates[index].result = solveNumericalIkDampedLeastSquares(
            candidates[index].seedRadians,
            targetPoseVector);

        candidates[index].score = scoreIkCandidate(candidates[index].result);

        if (candidates[index].score < bestScore)
        {
            bestScore = candidates[index].score;
            bestIndex = static_cast<int>(index);
        }
    }

    std::cout << "  Seed results:\n";
    for (const IkSeedCandidate& candidate : candidates)
    {
        printCompactIkCandidateLine(candidate);
    }

    if (bestIndex < 0)
    {
        std::cout << "  No IK candidates were evaluated.\n";
        return;
    }

    const IkSeedCandidate& best = candidates[bestIndex];

    std::cout << "\n  Best candidate summary:\n";
    std::cout << "    Seed name: " << best.name << "\n";
    std::cout << "    Converged: " << (best.result.converged ? "true" : "false") << "\n";
    std::cout << "    Hit joint limit: " << (best.result.hitJointLimit ? "true" : "false") << "\n";
    const double bestMaxDeltaDegrees = maxAbsJointDeltaDegrees(best.result.deltaRadians);
    const bool residualOk = residualAcceptedForDryRunCandidate(best.result);
    const bool jointDeltaOk = jointDeltaAcceptedForDryRunCandidate(best.result);
    const bool futureMotionCandidateOk = acceptedForFutureMotionDryRunCandidate(best.result);

    std::cout << "    Residual norm: " << std::fixed << std::setprecision(9) << best.result.residualNorm << "\n";
    std::cout << "    Max residual component: " << best.result.maxResidualComponent << "\n";
    std::cout << "    Max candidate joint delta: " << bestMaxDeltaDegrees << " degrees\n";
    std::cout << "    Residual gate <= " << CandidateResidualNormAccept << " norm and <= "
              << CandidateMaxResidualComponentAccept << " max component: "
              << (residualOk ? "PASS" : "FAIL") << "\n";
    std::cout << "    Joint-delta gate <= " << CandidateMaxJointDeltaDegreesAccept
              << " deg max joint delta: "
              << (jointDeltaOk ? "PASS" : "FAIL") << "\n";
    std::cout << "    Future motion dry-run candidate: "
              << (futureMotionCandidateOk ? "YES" : "NO") << "\n";
    printCartesianResidual("    Final residual", best.result.residual);

    printJointDegreesFromRadians("    Best seed joint degrees", best.seedRadians);
    printJointDegreesFromRadians("    Candidate joint degrees", best.result.solutionRadians);
    printJointDegreesFromRadians("    Candidate delta degrees", best.result.deltaRadians);
    printJointUserUnitsFromRadians("    Candidate delta user units/revolutions", best.result.deltaRadians);

    printJointLimitReport(best.result.solutionRadians);
    printOpenRaveFkReport("    FK verification for best multi-seed candidate", best.result.solutionRadians);

    if (futureMotionCandidateOk)
    {
        std::cout << "    Multi-seed IK verdict: candidate passes dry-run validation gates. Motion is still disabled in this patch.\n";
    }
    else
    {
        std::cout << "    Multi-seed IK verdict: no future-motion candidate yet. Do not command this target.\n";

        if (residualOk && !jointDeltaOk)
        {
            std::cout << "    Note: residual is small, but the required joint reconfiguration is too large for a tiny Cartesian test.\n";
            std::cout << "    This usually means the target is near a singular or symmetric pose and should be approached through a staged pre-bend/ready pose, not as one tiny Cartesian command from software zero.\n";
        }
    }
}


JointVector makeAllAxesVector(double value)
{
    JointVector values{};
    values.fill(value);
    return values;
}

JointVector negateJointVector(const JointVector& values)
{
    JointVector result{};

    for (size_t index = 0; index < result.size(); ++index)
    {
        result[index] = -values[index];
    }

    return result;
}

bool hasAnyNonZeroJoint(const JointVector& values)
{
    for (double value : values)
    {
        if (std::fabs(value) > 1e-12)
        {
            return true;
        }
    }

    return false;
}

double maxAbsJointValue(const JointVector& values)
{
    double result = 0.0;

    for (double value : values)
    {
        result = std::max(result, std::fabs(value));
    }

    return result;
}

static constexpr double TrajectoryMinPointSeconds = 0.050;
static constexpr int TrajectoryPvtEmptyCount = 2;

struct JointTrajectoryBlock
{
    std::vector<JointVector> positions;
    std::vector<JointVector> velocities;
    std::vector<double> times;
    double totalSeconds = 0.0;
};

bool sameNonZeroSign(double a, double b)
{
    return (a > 0.0 && b > 0.0) || (a < 0.0 && b < 0.0);
}

double samplePeriodSecondsFromController(RR::MotionController* controller)
{
    if (!controller)
    {
        return 0.001;
    }

    try
    {
        const double sampleRateHz = controller->SampleRateGet();

        if (std::isfinite(sampleRateHz) && sampleRateHz > 1.0)
        {
            return 1.0 / sampleRateHz;
        }
    }
    catch (const RR::RsiError&)
    {
    }
    catch (...)
    {
    }

    return 0.001;
}

double roundUpToSamplePeriod(double seconds, double samplePeriodSeconds)
{
    const double boundedSamplePeriod = std::max(samplePeriodSeconds, 0.001);
    const double boundedSeconds = std::max(seconds, boundedSamplePeriod);
    return std::ceil(boundedSeconds / boundedSamplePeriod) * boundedSamplePeriod;
}

JointTrajectoryBlock makePvtTrajectoryBlock(
    const JointVector& startingPosition,
    const std::vector<JointVector>& relativeSequence,
    double samplePeriodSeconds)
{
    if (relativeSequence.empty())
    {
        throw std::runtime_error("Cannot build a PVT trajectory block from an empty sequence.");
    }

    JointTrajectoryBlock block{};
    block.positions.reserve(relativeSequence.size());
    block.velocities.resize(relativeSequence.size());
    block.times.reserve(relativeSequence.size());

    std::vector<JointVector> segmentVelocities;
    segmentVelocities.reserve(relativeSequence.size());

    JointVector cumulativePosition = startingPosition;

    for (const JointVector& relativeStep : relativeSequence)
    {
        double pointSeconds = maxAbsJointValue(relativeStep) / std::max(MotionVelocity, 1e-9);
        pointSeconds = std::max(pointSeconds, TrajectoryMinPointSeconds);
        pointSeconds = roundUpToSamplePeriod(pointSeconds, samplePeriodSeconds);

        JointVector segmentVelocity{};

        for (int axis = 0; axis < Racer3BasicMotion::AxisCount; ++axis)
        {
            cumulativePosition[axis] += relativeStep[axis];
            segmentVelocity[axis] = relativeStep[axis] / pointSeconds;
        }

        block.positions.push_back(cumulativePosition);
        block.times.push_back(pointSeconds);
        block.totalSeconds += pointSeconds;
        segmentVelocities.push_back(segmentVelocity);
    }

    for (size_t pointIndex = 0; pointIndex < block.positions.size(); ++pointIndex)
    {
        JointVector waypointVelocity{};

        for (int axis = 0; axis < Racer3BasicMotion::AxisCount; ++axis)
        {
            if (pointIndex + 1 >= segmentVelocities.size())
            {
                waypointVelocity[axis] = 0.0;
                continue;
            }

            const double incomingVelocity = segmentVelocities[pointIndex][axis];
            const double outgoingVelocity = segmentVelocities[pointIndex + 1][axis];

            if (sameNonZeroSign(incomingVelocity, outgoingVelocity))
            {
                waypointVelocity[axis] = 0.5 * (incomingVelocity + outgoingVelocity);
            }
            else
            {
                waypointVelocity[axis] = 0.0;
            }
        }

        block.velocities[pointIndex] = waypointVelocity;
    }

    return block;
}

double smoothStepQuintic(double u)
{
    const double clamped = std::clamp(u, 0.0, 1.0);
    return clamped * clamped * clamped * (10.0 + clamped * (-15.0 + 6.0 * clamped));
}

double smoothStepQuinticDerivative(double u)
{
    const double clamped = std::clamp(u, 0.0, 1.0);
    return 30.0 * clamped * clamped * (1.0 - clamped) * (1.0 - clamped);
}

double endpointTrajectorySeconds(const JointVector& startPosition, const JointVector& targetPosition)
{
    JointVector delta{};

    for (int axis = 0; axis < Racer3BasicMotion::AxisCount; ++axis)
    {
        delta[axis] = targetPosition[axis] - startPosition[axis];
    }

    const double nominalSeconds = maxAbsJointValue(delta) / std::max(MotionVelocity, 1e-9);

    // This keeps the demo move visibly smooth even if the commanded delta is small.
    return std::max(1.5, nominalSeconds);
}

JointTrajectoryBlock makeSmoothEndpointPvtBlock(
    const JointVector& startingPosition,
    const JointVector& targetPosition,
    double samplePeriodSeconds)
{
    JointTrajectoryBlock block{};
    block.positions.reserve(EndpointPvtWaypointCount);
    block.velocities.reserve(EndpointPvtWaypointCount);
    block.times.reserve(EndpointPvtWaypointCount);

    const double totalSeconds =
        roundUpToSamplePeriod(
            endpointTrajectorySeconds(startingPosition, targetPosition),
            samplePeriodSeconds);

    const double pointSeconds =
        roundUpToSamplePeriod(
            totalSeconds / static_cast<double>(EndpointPvtWaypointCount),
            samplePeriodSeconds);

    JointVector delta{};

    for (int axis = 0; axis < Racer3BasicMotion::AxisCount; ++axis)
    {
        delta[axis] = targetPosition[axis] - startingPosition[axis];
    }

    for (int pointIndex = 1; pointIndex <= EndpointPvtWaypointCount; ++pointIndex)
    {
        const double u =
            static_cast<double>(pointIndex) /
            static_cast<double>(EndpointPvtWaypointCount);

        const double s = smoothStepQuintic(u);
        const double dsdu = smoothStepQuinticDerivative(u);

        JointVector position{};
        JointVector velocity{};

        for (int axis = 0; axis < Racer3BasicMotion::AxisCount; ++axis)
        {
            position[axis] = startingPosition[axis] + delta[axis] * s;
            velocity[axis] = delta[axis] * dsdu / totalSeconds;
        }

        // Force a true zero-velocity endpoint.
        if (pointIndex == EndpointPvtWaypointCount)
        {
            velocity.fill(0.0);
        }

        block.positions.push_back(position);
        block.velocities.push_back(velocity);
        block.times.push_back(pointSeconds);
        block.totalSeconds += pointSeconds;
    }

    return block;
}

std::vector<double> flattenJointTrajectoryPoints(const std::vector<JointVector>& points)
{
    std::vector<double> flattened;
    flattened.reserve(points.size() * Racer3BasicMotion::AxisCount);

    for (const JointVector& point : points)
    {
        for (double value : point)
        {
            flattened.push_back(value);
        }
    }

    return flattened;
}

double maxAbsTrajectoryVelocity(const JointTrajectoryBlock& block)
{
    double result = 0.0;

    for (const JointVector& waypointVelocity : block.velocities)
    {
        result = std::max(result, maxAbsJointValue(waypointVelocity));
    }

    return result;
}

double minTrajectoryPointSeconds(const JointTrajectoryBlock& block)
{
    if (block.times.empty())
    {
        return 0.0;
    }

    return *std::min_element(block.times.begin(), block.times.end());
}

double maxTrajectoryPointSeconds(const JointTrajectoryBlock& block)
{
    if (block.times.empty())
    {
        return 0.0;
    }

    return *std::max_element(block.times.begin(), block.times.end());
}

int trajectoryBlockMotionTimeoutMs(const JointTrajectoryBlock& block)
{
    const int64_t estimatedMilliseconds =
        static_cast<int64_t>(std::ceil((block.totalSeconds * 4.0 * 1000.0) + 30000.0));
    const int64_t timeoutMilliseconds =
        std::min<int64_t>(
            15 * 60 * 1000,
            std::max<int64_t>(MotionTimeoutMs, estimatedMilliseconds));

    return static_cast<int>(timeoutMilliseconds);
}

int queuedSequenceMotionTimeoutMs(const std::vector<JointVector>& sequence)
{
    double nominalSeconds = 0.0;
    const double velocity = std::max(MotionVelocity, 1e-9);

    for (const JointVector& step : sequence)
    {
        nominalSeconds += maxAbsJointValue(step) / velocity;
    }

    const int64_t estimatedMilliseconds =
        static_cast<int64_t>(std::ceil((nominalSeconds * 6.0 * 1000.0) + 30000.0));
    const int64_t timeoutMilliseconds =
        std::min<int64_t>(
            15 * 60 * 1000,
            std::max<int64_t>(MotionTimeoutMs, estimatedMilliseconds));

    return static_cast<int>(timeoutMilliseconds);
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

const char* const RobotProbeAxisLabels[Racer3BasicMotion::AxisCount] = {
    "X-Axis",
    "Y-Axis",
    "Z-Axis",
    "Roll-Axis",
    "Pitch-Axis",
    "Yaw-Axis"
};

void setRobotProbeAxisLabels(std::array<RR::Axis*, Racer3BasicMotion::AxisCount>& axes)
{
    std::cout << "Setting temporary Axis UserLabel values for Cartesian Robot validation:\n";

    for (int index = 0; index < Racer3BasicMotion::AxisCount; ++index)
    {
        if (!axes[index])
        {
            throw std::runtime_error("Axis " + std::to_string(index + 1) + " is not initialized.");
        }

        axes[index]->UserLabelSet(RobotProbeAxisLabels[index]);
        std::cout << "  Axis " << (index + 1)
                  << " UserLabelSet(\""
                  << RobotProbeAxisLabels[index]
                  << "\")\n";
    }
}

template <size_t N>
void setExpectedLabel(char (&destination)[N], const char* source)
{
    std::snprintf(destination, N, "%s", source);
}

void addSixAxisXyzAbcLinearJoints(RC::LinearModelBuilder& builder)
{
    const double scaling = 1.0;
    const double offset = 0.0;

    RC::LinearJointMapping x(0, RC::CartesianAxis::X);
    setExpectedLabel(x.ExpectedLabel, RobotProbeAxisLabels[0]);
    x.Scaling = scaling;
    x.Offset = offset;
    builder.JointAdd(x);

    RC::LinearJointMapping y(1, RC::CartesianAxis::Y);
    setExpectedLabel(y.ExpectedLabel, RobotProbeAxisLabels[1]);
    y.Scaling = scaling;
    y.Offset = offset;
    builder.JointAdd(y);

    RC::LinearJointMapping z(2, RC::CartesianAxis::Z);
    setExpectedLabel(z.ExpectedLabel, RobotProbeAxisLabels[2]);
    z.Scaling = scaling;
    z.Offset = offset;
    builder.JointAdd(z);

    RC::LinearJointMapping roll(3, RC::CartesianAxis::Roll);
    setExpectedLabel(roll.ExpectedLabel, RobotProbeAxisLabels[3]);
    roll.Scaling = scaling;
    roll.Offset = offset;
    builder.JointAdd(roll);

    RC::LinearJointMapping pitch(4, RC::CartesianAxis::Pitch);
    setExpectedLabel(pitch.ExpectedLabel, RobotProbeAxisLabels[4]);
    pitch.Scaling = scaling;
    pitch.Offset = offset;
    builder.JointAdd(pitch);

    RC::LinearJointMapping yaw(5, RC::CartesianAxis::Yaw);
    setExpectedLabel(yaw.ExpectedLabel, RobotProbeAxisLabels[5]);
    yaw.Scaling = scaling;
    yaw.Offset = offset;
    builder.JointAdd(yaw);
}

void probeConfiguredLinearBuilder(
    RR::MotionController* controller,
    RR::MultiAxis* multiAxis,
    const char* modelLabel,
    const char* probeLabel)
{
    std::cout << "\n" << probeLabel << ": LinearModelBuilder(\""
              << modelLabel
              << "\") + X/Y/Z/Roll/Pitch/Yaw JointAdd mappings + RobotCreate(...)\n";

    try
    {
        RC::LinearModelBuilder builder(modelLabel);
        std::cout << "  LinearModelBuilder constructed.\n";

        addSixAxisXyzAbcLinearJoints(builder);
        std::cout << "  Added six JointAdd mappings with ExpectedLabel values matching Axis UserLabel values.\n";

        const RC::KinematicModel& model = builder.ModelBuild();
        (void)model;
        std::cout << "  ModelBuild returned a KinematicModel reference.\n";

        RC::Robot* robot = RC::Robot::RobotCreate(controller, multiAxis, &builder);
        if (!robot)
        {
            std::cout << "  RobotCreate returned null.\n";
            return;
        }

        std::cout << "  RobotCreate returned a non-null Robot pointer.\n";
        printErrorLog("  Robot(configured builder)", robot);
        RC::Robot::RobotDelete(controller, robot);
        std::cout << "  RobotDelete completed.\n";
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "  " << probeLabel << " RapidCode error: " << error.text << "\n";
        std::cout << "  Function: " << error.functionName << "\n";
    }
    catch (const std::exception& error)
    {
        std::cout << "  " << probeLabel << " std::exception: " << error.what() << "\n";
    }
    catch (...)
    {
        std::cout << "  " << probeLabel << " unknown exception.\n";
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
    DiagnosticsEnabled = options.diagnostics;
    DualMotionEnabled = options.dualMotion;
    AllMotionEnabled = options.allMotion;
    JointVectorMotionEnabled = options.jointVectorMotion;
    RobotModelProbeEnabled = options.robotModelProbe;
    RobotPoseProbeEnabled = options.robotPoseProbe;
    KinematicsDryRunEnabled = options.kinematicsDryRun;
    CartesianVectorMotionEnabled = options.cartesianVectorMotion;
    PositionOnlyIkEnabled = options.positionOnlyIk;
    CompactSegmentedExecutionEnabled = options.compactMotion;
    AppendSegmentedExecutionEnabled = options.appendMotion;
    TrajectorySegmentedExecutionEnabled = options.trajectoryMotion;
    EndpointOnlyMotionEnabled = options.endpointOnlyMotion;
    SegmentGoalMotionEnabled = options.segmentGoalMotion;
    CartesianVectorMotionConfirmed = options.motionConfirmed;
    ReturnWarnToleranceUserUnits = options.returnWarnToleranceUserUnits;
    ReturnFailToleranceUserUnits = options.returnFailToleranceUserUnits;
    RequestedJointVector = options.jointVectorUserUnits;
    RequestedCartesianVector = options.cartesianVector;

    if (ReturnWarnToleranceUserUnits < 0.0)
    {
        throw std::runtime_error("--return-warn must be zero or greater.");
    }

    if (ReturnFailToleranceUserUnits <= 0.0)
    {
        throw std::runtime_error("--return-fail must be greater than zero.");
    }

    if (ReturnWarnToleranceUserUnits > ReturnFailToleranceUserUnits)
    {
        throw std::runtime_error("--return-warn must be less than or equal to --return-fail.");
    }

    if (!JointVectorMotionEnabled && options.stepUserUnits <= 0.0)
    {
        throw std::runtime_error("--step must be greater than zero.");
    }

    if (options.velocityUserUnitsPerSecond <= 0.0)
    {
        throw std::runtime_error("--velocity must be greater than zero.");
    }

    Axis6TestStepUserUnits = options.stepUserUnits;
    MotionVelocity = options.velocityUserUnitsPerSecond;

    if (JointVectorMotionEnabled && !hasAnyNonZeroJoint(RequestedJointVector))
    {
        throw std::runtime_error("--joints must contain at least one nonzero joint value for --joint-vector.");
    }

    if (CartesianVectorMotionEnabled && !hasAnyNonZeroJoint(RequestedCartesianVector))
    {
        throw std::runtime_error("--cartesian must contain at least one nonzero value for --cartesian-vector.");
    }

    if (AppendSegmentedExecutionEnabled)
    {
        if (!CartesianVectorMotionEnabled)
        {
            throw std::runtime_error("--append-motion requires --cartesian-vector.");
        }

        if (!PositionOnlyIkEnabled)
        {
            throw std::runtime_error("--append-motion is experimental and currently requires --position-only.");
        }

    }

    if (TrajectorySegmentedExecutionEnabled)
    {
        if (!CartesianVectorMotionEnabled)
        {
            throw std::runtime_error("--trajectory-motion requires --cartesian-vector.");
        }

        if (!PositionOnlyIkEnabled)
        {
            throw std::runtime_error("--trajectory-motion is experimental and currently requires --position-only.");
        }

        if (AppendSegmentedExecutionEnabled)
        {
            throw std::runtime_error("--trajectory-motion cannot be combined with --append-motion.");
        }
    }

    if (EndpointOnlyMotionEnabled)
    {
        if (!CartesianVectorMotionEnabled)
        {
            throw std::runtime_error("--endpoint-only requires --cartesian-vector.");
        }

        if (!PositionOnlyIkEnabled)
        {
            throw std::runtime_error("--endpoint-only currently requires --position-only.");
        }

        if (AppendSegmentedExecutionEnabled)
        {
            throw std::runtime_error("--endpoint-only cannot be combined with --append-motion.");
        }
    }

    if (SegmentGoalMotionEnabled)
    {
        if (!CartesianVectorMotionEnabled)
        {
            throw std::runtime_error("--segment-goal requires --cartesian-vector.");
        }

        if (!PositionOnlyIkEnabled)
        {
            throw std::runtime_error("--segment-goal currently requires --position-only.");
        }

        if (AppendSegmentedExecutionEnabled)
        {
            throw std::runtime_error("--segment-goal cannot be combined with --append-motion.");
        }

        if (EndpointOnlyMotionEnabled)
        {
            throw std::runtime_error("--segment-goal cannot be combined with --endpoint-only.");
        }
    }

    printMotionPlan();

    if (options.dryRun)
    {
        std::cout << "Dry run complete. No RMP calls were made.\n";
        return;
    }

    try
    {
        connectController();

        if (options.robotModelProbe)
        {
            runRobotModelProbe();
            safeShutdown();
            return;
        }

        if (options.robotPoseProbe)
        {
            runRobotPoseProbe();
            safeShutdown();
            return;
        }

        if (options.kinematicsDryRun)
        {
            runKinematicsDryRun();
            safeShutdown();
            return;
        }

        if (options.cartesianVectorMotion)
        {
            runCartesianVectorMotion();
            safeShutdown();
            return;
        }

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
        else if (options.dualMotion)
        {
            runDualAxisMotion();
        }
        else if (options.allMotion)
        {
            runAllAxisMotion();
        }
        else if (options.jointVectorMotion)
        {
            runJointVectorMotion();
        }

        if (options.tinyMotion || options.dualMotion || options.allMotion || options.jointVectorMotion || options.cartesianVectorMotion)
        {
            printReturnToZeroReport("Return-to-zero check after motion", true);
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


    if (DualMotionEnabled)
    {
        std::cout << "Configuring Axis 5 / J5 for one-revolution user units for dual-axis mode...\n";

        if (!axes_[Axis5Index])
        {
            throw std::runtime_error("Axis 5 / J5 is not initialized.");
        }

        axes_[Axis5Index]->UserUnitsSet(Axis5CountsPerRevolution);
        axes_[Axis5Index]->PositionSet(0.0);
        axes_[Axis5Index]->HomeActionSet(RR::RSIAction::RSIActionNONE);
        axes_[Axis5Index]->PositionToleranceFineSet(Axis6FineTolerance);
        axes_[Axis5Index]->PositionToleranceCoarseSet(Axis6CoarseTolerance);
        axes_[Axis5Index]->VelocityToleranceSet(Axis6VelocityTolerance);
        axes_[Axis5Index]->SettlingTimeSet(Axis6SettlingTime);

        std::cout << "Axis 5 / J5 user units set: 1.0 user unit = 1 physical revolution.\n";
        std::cout << "Axis 5 / J5 current position set to software zero.\n";
        std::cout << "Axis 5 / J5 HomeActionSet(RSIActionNONE) applied.\n";
    }

    if (AllMotionEnabled || JointVectorMotionEnabled || RobotPoseProbeEnabled || KinematicsDryRunEnabled || CartesianVectorMotionEnabled)
    {
        configureAllAxesForAllMotion();
    }

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


void Racer3BasicMotion::configureAxis5MotionAttributes(const char* context)
{
    if (!axes_[Axis5Index])
    {
        throw std::runtime_error("Axis 5 / J5 is not initialized.");
    }

    std::cout << "Resetting Axis 5 motion attributes (" << context << ")...\n";

    axes_[Axis5Index]->MotionAttributeMaskDefaultSet();
    axes_[Axis5Index]->MotionDelaySet(0.0);
    axes_[Axis5Index]->FeedRateSet(1.0);

    printMotionAttributeMasks("Axis 5", axes_[Axis5Index]);
}


void Racer3BasicMotion::configureAxisMotionAttributes(int axisIndex, const char* context)
{
    if (axisIndex < 0 || axisIndex >= AxisCount || !axes_[axisIndex])
    {
        throw std::runtime_error("Axis is not initialized for configureAxisMotionAttributes.");
    }

    std::cout << "Resetting Axis " << (axisIndex + 1) << " motion attributes (" << context << ")...\n";

    axes_[axisIndex]->MotionAttributeMaskDefaultSet();
    axes_[axisIndex]->MotionDelaySet(0.0);
    axes_[axisIndex]->FeedRateSet(1.0);

    printMotionAttributeMasks(("Axis " + std::to_string(axisIndex + 1)).c_str(), axes_[axisIndex]);
}

void Racer3BasicMotion::configureAllAxesForAllMotion()
{
    std::cout << "Configuring all six axes for one-revolution user units for all-axis mode...\n";

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            throw std::runtime_error("Axis " + std::to_string(index + 1) + " is not initialized.");
        }

        axes_[index]->UserUnitsSet(Racer3CountsPerRevolution[index]);
        axes_[index]->PositionSet(0.0);
        axes_[index]->HomeActionSet(RR::RSIAction::RSIActionNONE);
        axes_[index]->PositionToleranceFineSet(Axis6FineTolerance);
        axes_[index]->PositionToleranceCoarseSet(Axis6CoarseTolerance);
        axes_[index]->VelocityToleranceSet(Axis6VelocityTolerance);
        axes_[index]->SettlingTimeSet(Axis6SettlingTime);

        std::cout << "  Axis " << (index + 1)
                  << " counts/user-unit=" << std::fixed << std::setprecision(0)
                  << Racer3CountsPerRevolution[index]
                  << ", HomeAction=NONE, software zero set.\n";
    }

    std::cout << std::fixed << std::setprecision(6);
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


void Racer3BasicMotion::isolateAxis5And6ForDualMotion()
{
    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    if (!axes_[Axis5Index] || !axes_[Axis6Index])
    {
        throw std::runtime_error("Axis 5 / J5 or Axis 6 / J6 is not initialized.");
    }

    std::cout << "Isolating Axis 5 and Axis 6 for synchronized MultiAxis motion...\n";
    std::cout << "All six drives have already been enabled; MultiAxis 6 will now contain only J5 and J6.\n";

    multiAxis_->AxisRemoveAll();
    multiAxis_->AxisAdd(axes_[Axis5Index]);
    multiAxis_->AxisAdd(axes_[Axis6Index]);
    multiAxis_->UserLabelSet("Racer3J5J6DualDemo");

    std::this_thread::sleep_for(std::chrono::milliseconds(EnableSettleMs));

    configureMultiAxisMotionAttributes("after remapping MultiAxis 6 to only Axis 5 and Axis 6");
    configureAxis5MotionAttributes("after dual-axis MultiAxis remap");
    configureAxis6MotionAttributes("after dual-axis MultiAxis remap");

    std::cout << "Clearing Axis 5/6 faults and confirming MultiAxis 6 amp enable for dual motion...\n";
    axes_[Axis5Index]->ClearFaults();
    axes_[Axis6Index]->ClearFaults();
    multiAxis_->ClearFaults();

    const int result = multiAxis_->AmpEnableSet(
        true,
        AmpEnableTimeoutMs,
        OverrideRestrictedStateForEnable);

    if (!multiAxis_->AmpEnableGet() || !axes_[Axis5Index]->AmpEnableGet() || !axes_[Axis6Index]->AmpEnableGet())
    {
        throw std::runtime_error("Axis 5/6 MultiAxis AmpEnableSet failed or timed out after dual-axis isolation.");
    }

    std::cout << "Axis 5/6 MultiAxis amp enable confirmed after isolation. AmpEnableSet returned "
              << result
              << " ms.\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(EnableSettleMs));
    printAxis5And6MotionStatus("Axis 5/6 status after dual-axis isolation");
}


void Racer3BasicMotion::isolateAllAxesForAllMotion()
{
    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    std::cout << "Preparing MultiAxis 6 for synchronized all-axis motion...\n";
    std::cout << "All six drives have already been enabled; MultiAxis 6 will now contain Axis 1 through Axis 6.\n";

    multiAxis_->AxisRemoveAll();

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            throw std::runtime_error("Axis " + std::to_string(index + 1) + " is not initialized.");
        }

        multiAxis_->AxisAdd(axes_[index]);
        std::cout << "  Added Axis " << (index + 1) << " to all-axis MultiAxis group.\n";
    }

    multiAxis_->UserLabelSet("Racer3AllAxisDemo");

    std::this_thread::sleep_for(std::chrono::milliseconds(EnableSettleMs));

    configureMultiAxisMotionAttributes("after remapping MultiAxis 6 to all six axes");

    for (int index = 0; index < AxisCount; ++index)
    {
        configureAxisMotionAttributes(index, "after all-axis MultiAxis remap");
    }

    std::cout << "Clearing all axis faults and confirming MultiAxis 6 amp enable for all-axis motion...\n";

    for (int index = 0; index < AxisCount; ++index)
    {
        axes_[index]->ClearFaults();
    }

    multiAxis_->ClearFaults();

    const int result = multiAxis_->AmpEnableSet(
        true,
        AmpEnableTimeoutMs,
        OverrideRestrictedStateForEnable);

    if (!multiAxis_->AmpEnableGet())
    {
        throw std::runtime_error("All-axis MultiAxis AmpEnableSet failed or timed out after all-axis isolation.");
    }

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index]->AmpEnableGet())
        {
            throw std::runtime_error("Axis " + std::to_string(index + 1) + " amp enable failed after all-axis isolation.");
        }
    }

    std::cout << "All-axis MultiAxis amp enable confirmed after isolation. AmpEnableSet returned "
              << result
              << " ms.\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(EnableSettleMs));
    printAllAxisMotionStatus("All-axis status after all-axis isolation");
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


void Racer3BasicMotion::runDualAxisMotion()
{
    std::cout << "Starting Axis 5 + Axis 6 synchronized MultiAxis::MoveRelative diagnostic...\n";
    std::cout << "All 6 axes were enabled through runtime-mapped MultiAxis 6 first.\n";
    std::cout << "Before the dual move, MultiAxis 6 is remapped to contain only Axis 5 / J5 and Axis 6 / J6.\n";
    std::cout << "Both axes receive the same relative range and velocity so they complete at the same time.\n";
    std::cout << "Step = "
              << Axis6TestStepUserUnits
              << " user units = "
              << toDegrees(Axis6TestStepUserUnits)
              << " degrees on each axis.\n";
    std::cout << "Velocity = "
              << MotionVelocity
              << " user-units/sec = "
              << toDegrees(MotionVelocity)
              << " deg/sec on each axis.\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    if (!axes_[Axis5Index] || !axes_[Axis6Index])
    {
        throw std::runtime_error("Axis 5 / J5 or Axis 6 / J6 is not initialized.");
    }

    isolateAxis5And6ForDualMotion();

    const RR::RSIAction originalAxis5ErrorLimitAction = axes_[Axis5Index]->ErrorLimitActionGet();
    const RR::RSIAction originalAxis6ErrorLimitAction = axes_[Axis6Index]->ErrorLimitActionGet();
    bool errorLimitsTemporarilyChanged = false;

    if (TemporarilyDisableAxis6ErrorLimitForTinyMotion)
    {
        std::cout << "Temporarily setting Axis 5 and Axis 6 position ErrorLimitAction to RSIActionNONE for this dual-axis motion test.\n";
        std::cout << "  Original Axis 5 ErrorLimitAction: " << actionName(originalAxis5ErrorLimitAction) << "\n";
        std::cout << "  Original Axis 6 ErrorLimitAction: " << actionName(originalAxis6ErrorLimitAction) << "\n";
        std::cout << "  Amp fault and hardware limit actions are not changed.\n";
        axes_[Axis5Index]->ErrorLimitActionSet(RR::RSIAction::RSIActionNONE);
        axes_[Axis6Index]->ErrorLimitActionSet(RR::RSIAction::RSIActionNONE);
        errorLimitsTemporarilyChanged = true;
    }

    // MultiAxis 6 currently contains exactly two axes, in this order:
    //   array index 0 -> Axis 5 / J5
    //   array index 1 -> Axis 6 / J6
    const std::array<std::array<double, 2>, 2> relativePositions = {{
        {{ Axis6TestStepUserUnits, Axis6TestStepUserUnits }},
        {{ -Axis6TestStepUserUnits, -Axis6TestStepUserUnits }}
    }};

    const std::array<double, 2> velocity = {{ MotionVelocity, MotionVelocity }};
    const std::array<double, 2> acceleration = {{ MotionAcceleration, MotionAcceleration }};
    const std::array<double, 2> deceleration = {{ MotionDeceleration, MotionDeceleration }};
    const std::array<double, 2> jerk = {{ MotionJerkPercent, MotionJerkPercent }};

    try
    {
        for (size_t stepIndex = 0; stepIndex < relativePositions.size(); ++stepIndex)
        {
            const auto& relativePosition = relativePositions[stepIndex];

            std::cout << "\n=== Axis 5 + Axis 6 MultiAxis::MoveRelative step "
                      << (stepIndex + 1)
                      << " / "
                      << relativePositions.size()
                      << " ===\n";

            std::cout << "Relative position array [J5, J6]: "
                      << relativePosition[0]
                      << ", "
                      << relativePosition[1]
                      << " user units = "
                      << toDegrees(relativePosition[0])
                      << ", "
                      << toDegrees(relativePosition[1])
                      << " degrees.\n";
            std::cout << "Velocity array [J5, J6]: "
                      << velocity[0]
                      << ", "
                      << velocity[1]
                      << " user-units/sec.\n";

            clearErrorLog("MotionController", controller_);
            clearErrorLog("MultiAxis 6", multiAxis_);
            clearErrorLog("Axis 5", axes_[Axis5Index]);
            clearErrorLog("Axis 6", axes_[Axis6Index]);

            configureMultiAxisMotionAttributes("before dual Axis 5/6 MultiAxis::MoveRelative step");
            configureAxis5MotionAttributes("before dual Axis 5/6 MultiAxis::MoveRelative step");
            configureAxis6MotionAttributes("before dual Axis 5/6 MultiAxis::MoveRelative step");
            printAxis5And6MotionStatus("Axis 5/6 before dual MoveRelative");

            const uint16_t commandedMotionId = multiAxis_->MotionIdGet();
            const double startingAxis5CommandPosition = axes_[Axis5Index]->CommandPositionGet();
            const double startingAxis6CommandPosition = axes_[Axis6Index]->CommandPositionGet();

            std::cout << "Commanding MultiAxis::MoveRelative for [Axis5, Axis6].\n";
            std::cout << "  MultiAxis commanded MotionId before call: " << commandedMotionId << "\n";

            multiAxis_->MoveRelative(
                relativePosition.data(),
                velocity.data(),
                acceleration.data(),
                deceleration.data(),
                jerk.data());

            std::cout << "  MultiAxis next MotionId after call: " << multiAxis_->MotionIdGet() << "\n";
            printAxis5And6MotionStatus("Axis 5/6 immediately after dual MoveRelative");

            waitForDualAxisMotionStart(
                "Axis 5/6 MultiAxis::MoveRelative",
                startingAxis5CommandPosition,
                startingAxis6CommandPosition);

            for (int sample = 0; sample < 8; ++sample)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(MotionStatusSampleMs));
                printDualAxisProgressLine("Dual live sample", sample + 1);
            }

            waitForMotionDone(MotionTimeoutMs);

            printActualPositions("Actual positions after dual Axis 5/6 MultiAxis::MoveRelative step");
            printAxis5And6MotionStatus("Axis 5/6 after MotionDoneWait");
        }
    }
    catch (...)
    {
        if (errorLimitsTemporarilyChanged)
        {
            axes_[Axis5Index]->ErrorLimitActionSet(originalAxis5ErrorLimitAction);
            axes_[Axis6Index]->ErrorLimitActionSet(originalAxis6ErrorLimitAction);
            std::cout << "Restored Axis 5 ErrorLimitAction to " << actionName(originalAxis5ErrorLimitAction) << ".\n";
            std::cout << "Restored Axis 6 ErrorLimitAction to " << actionName(originalAxis6ErrorLimitAction) << ".\n";
        }

        throw;
    }

    if (errorLimitsTemporarilyChanged)
    {
        axes_[Axis5Index]->ErrorLimitActionSet(originalAxis5ErrorLimitAction);
        axes_[Axis6Index]->ErrorLimitActionSet(originalAxis6ErrorLimitAction);
        std::cout << "Restored Axis 5 ErrorLimitAction to " << actionName(originalAxis5ErrorLimitAction) << ".\n";
        std::cout << "Restored Axis 6 ErrorLimitAction to " << actionName(originalAxis6ErrorLimitAction) << ".\n";
    }

    std::cout << "Axis 5 + Axis 6 synchronized MultiAxis MoveRelative diagnostic complete. Net commanded offsets are zero.\n";
}


void Racer3BasicMotion::runAllAxisMotion()
{
    std::cout << "Starting synchronized all-axis MultiAxis::MoveRelative diagnostic...\n";
    std::cout << "All 6 axes were enabled through runtime-mapped MultiAxis 6 first.\n";
    std::cout << "Before the all-axis move, MultiAxis 6 is remapped to contain Axis 1 through Axis 6.\n";
    std::cout << "All axes receive the same relative range and velocity so they complete at the same time.\n";
    std::cout << "Step = "
              << Axis6TestStepUserUnits
              << " user units = "
              << toDegrees(Axis6TestStepUserUnits)
              << " degrees on each axis.\n";
    std::cout << "Velocity = "
              << MotionVelocity
              << " user-units/sec = "
              << toDegrees(MotionVelocity)
              << " deg/sec on each axis.\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            throw std::runtime_error("Axis " + std::to_string(index + 1) + " is not initialized.");
        }
    }

    isolateAllAxesForAllMotion();

    std::array<RR::RSIAction, AxisCount> originalErrorLimitActions{};
    bool errorLimitsTemporarilyChanged = false;

    if (TemporarilyDisableAxis6ErrorLimitForTinyMotion)
    {
        std::cout << "Temporarily setting all axes position ErrorLimitAction to RSIActionNONE for this all-axis motion test.\n";
        std::cout << "  Amp fault and hardware limit actions are not changed.\n";

        for (int index = 0; index < AxisCount; ++index)
        {
            originalErrorLimitActions[index] = axes_[index]->ErrorLimitActionGet();
            std::cout << "  Original Axis " << (index + 1)
                      << " ErrorLimitAction: " << actionName(originalErrorLimitActions[index]) << "\n";
            axes_[index]->ErrorLimitActionSet(RR::RSIAction::RSIActionNONE);
        }

        errorLimitsTemporarilyChanged = true;
    }

    const std::array<JointVector, 2> relativePositions = {
        makeAllAxesVector(Axis6TestStepUserUnits),
        makeAllAxesVector(-Axis6TestStepUserUnits)
    };

    const JointVector velocity = makeAllAxesVector(MotionVelocity);
    const JointVector acceleration = makeAllAxesVector(MotionAcceleration);
    const JointVector deceleration = makeAllAxesVector(MotionDeceleration);
    const JointVector jerk = makeAllAxesVector(MotionJerkPercent);

    if (CompactSegmentedExecutionEnabled)
    {
        std::cout << "CompactMotion: clearing error logs once before the segmented sequence.\n";
        clearErrorLog("MotionController", controller_);
        clearErrorLog("MultiAxis 6", multiAxis_);

        for (int index = 0; index < AxisCount; ++index)
        {
            clearErrorLog(("Axis " + std::to_string(index + 1)).c_str(), axes_[index]);
        }
    }

    try
    {
        for (size_t stepIndex = 0; stepIndex < relativePositions.size(); ++stepIndex)
        {
            const JointVector& relativePosition = relativePositions[stepIndex];

            std::cout << "\n=== All-axis MultiAxis::MoveRelative step "
                      << (stepIndex + 1)
                      << " / "
                      << relativePositions.size()
                      << " ===\n";

            std::cout << "Relative position array [J1..J6] in user units: ";
            printJointVector(relativePosition);
            std::cout << "Velocity array [J1..J6] in user-units/sec: ";
            printJointVector(velocity);

            clearErrorLog("MotionController", controller_);
            clearErrorLog("MultiAxis 6", multiAxis_);

            for (int index = 0; index < AxisCount; ++index)
            {
                clearErrorLog(("Axis " + std::to_string(index + 1)).c_str(), axes_[index]);
            }

            configureMultiAxisMotionAttributes("before all-axis MultiAxis::MoveRelative step");

            for (int index = 0; index < AxisCount; ++index)
            {
                configureAxisMotionAttributes(index, "before all-axis MultiAxis::MoveRelative step");
            }

            printAllAxisMotionStatus("All axes before all-axis MoveRelative");

            const uint16_t commandedMotionId = multiAxis_->MotionIdGet();
            std::array<double, AxisCount> startingCommandPositions{};

            for (int index = 0; index < AxisCount; ++index)
            {
                startingCommandPositions[index] = axes_[index]->CommandPositionGet();
            }

            std::cout << "Commanding MultiAxis::MoveRelative for [Axis1..Axis6].\n";
            std::cout << "  MultiAxis commanded MotionId before call: " << commandedMotionId << "\n";

            multiAxis_->MoveRelative(
                relativePosition.data(),
                velocity.data(),
                acceleration.data(),
                deceleration.data(),
                jerk.data());

            std::cout << "  MultiAxis next MotionId after call: " << multiAxis_->MotionIdGet() << "\n";
            printAllAxisMotionStatus("All axes immediately after all-axis MoveRelative");

            waitForAllAxisMotionStart(
                "All-axis MultiAxis::MoveRelative",
                startingCommandPositions);

            for (int sample = 0; sample < 8; ++sample)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(MotionStatusSampleMs));
                printAllAxisProgressLine("All-axis live sample", sample + 1);
            }

            waitForMotionDone(MotionTimeoutMs);

            printActualPositions("Actual positions after all-axis MultiAxis::MoveRelative step");
            printAllAxisMotionStatus("All axes after MotionDoneWait");
        }
    }
    catch (...)
    {
        if (errorLimitsTemporarilyChanged)
        {
            for (int index = 0; index < AxisCount; ++index)
            {
                axes_[index]->ErrorLimitActionSet(originalErrorLimitActions[index]);
                std::cout << "Restored Axis " << (index + 1)
                          << " ErrorLimitAction to " << actionName(originalErrorLimitActions[index]) << ".\n";
            }
        }

        throw;
    }

    if (errorLimitsTemporarilyChanged)
    {
        for (int index = 0; index < AxisCount; ++index)
        {
            axes_[index]->ErrorLimitActionSet(originalErrorLimitActions[index]);
            std::cout << "Restored Axis " << (index + 1)
                      << " ErrorLimitAction to " << actionName(originalErrorLimitActions[index]) << ".\n";
        }
    }

    std::cout << "Synchronized all-axis MultiAxis MoveRelative diagnostic complete. Net commanded offsets are zero.\n";
}


void Racer3BasicMotion::runJointVectorMotion()
{
    std::cout << "Starting custom joint-vector MultiAxis::MoveRelative diagnostic...\n";
    std::cout << "All 6 axes were enabled through runtime-mapped MultiAxis 6 first.\n";
    std::cout << "Before the joint-vector move, MultiAxis 6 is remapped to Axis 1 through Axis 6.\n";
    std::cout << "The requested relative joint vector is commanded, then its negative is commanded to return to software zero.\n";
    std::cout << "Max requested joint delta = "
              << maxAbsJointValue(RequestedJointVector)
              << " user units = "
              << toDegrees(maxAbsJointValue(RequestedJointVector))
              << " degrees.\n";
    std::cout << "Velocity = "
              << MotionVelocity
              << " user-units/sec = "
              << toDegrees(MotionVelocity)
              << " deg/sec on each axis.\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            throw std::runtime_error("Axis " + std::to_string(index + 1) + " is not initialized.");
        }
    }

    isolateAllAxesForAllMotion();

    std::array<RR::RSIAction, AxisCount> originalErrorLimitActions{};
    bool errorLimitsTemporarilyChanged = false;

    if (TemporarilyDisableAxis6ErrorLimitForTinyMotion)
    {
        std::cout << "Temporarily setting all axes position ErrorLimitAction to RSIActionNONE for this joint-vector motion test.\n";
        std::cout << "  Amp fault and hardware limit actions are not changed.\n";

        for (int index = 0; index < AxisCount; ++index)
        {
            originalErrorLimitActions[index] = axes_[index]->ErrorLimitActionGet();
            std::cout << "  Original Axis " << (index + 1)
                      << " ErrorLimitAction: " << actionName(originalErrorLimitActions[index]) << "\n";
            axes_[index]->ErrorLimitActionSet(RR::RSIAction::RSIActionNONE);
        }

        errorLimitsTemporarilyChanged = true;
    }

    const std::array<JointVector, 2> relativePositions = {
        RequestedJointVector,
        negateJointVector(RequestedJointVector)
    };

    const JointVector velocity = makeAllAxesVector(MotionVelocity);
    const JointVector acceleration = makeAllAxesVector(MotionAcceleration);
    const JointVector deceleration = makeAllAxesVector(MotionDeceleration);
    const JointVector jerk = makeAllAxesVector(MotionJerkPercent);

    try
    {
        for (size_t stepIndex = 0; stepIndex < relativePositions.size(); ++stepIndex)
        {
            const JointVector& relativePosition = relativePositions[stepIndex];

            std::cout << "\n=== Joint-vector MultiAxis::MoveRelative step "
                      << (stepIndex + 1)
                      << " / "
                      << relativePositions.size()
                      << " ===\n";

            std::cout << "Relative position array [J1..J6] in user units: ";
            printJointVector(relativePosition);
            std::cout << "Relative position array [J1..J6] in approx degrees: ";
            JointVector degrees{};
            for (size_t axis = 0; axis < relativePosition.size(); ++axis)
            {
                degrees[axis] = toDegrees(relativePosition[axis]);
            }
            printJointVector(degrees);
            std::cout << "Velocity array [J1..J6] in user-units/sec: ";
            printJointVector(velocity);

            clearErrorLog("MotionController", controller_);
            clearErrorLog("MultiAxis 6", multiAxis_);

            for (int index = 0; index < AxisCount; ++index)
            {
                clearErrorLog(("Axis " + std::to_string(index + 1)).c_str(), axes_[index]);
            }

            configureMultiAxisMotionAttributes("before joint-vector MultiAxis::MoveRelative step");

            for (int index = 0; index < AxisCount; ++index)
            {
                configureAxisMotionAttributes(index, "before joint-vector MultiAxis::MoveRelative step");
            }

            printAllAxisMotionStatus("All axes before joint-vector MoveRelative");

            const uint16_t commandedMotionId = multiAxis_->MotionIdGet();
            std::array<double, AxisCount> startingCommandPositions{};

            for (int index = 0; index < AxisCount; ++index)
            {
                startingCommandPositions[index] = axes_[index]->CommandPositionGet();
            }

            std::cout << "Commanding MultiAxis::MoveRelative for custom [Axis1..Axis6] joint vector.\n";
            std::cout << "  MultiAxis commanded MotionId before call: " << commandedMotionId << "\n";

            multiAxis_->MoveRelative(
                relativePosition.data(),
                velocity.data(),
                acceleration.data(),
                deceleration.data(),
                jerk.data());

            std::cout << "  MultiAxis next MotionId after call: " << multiAxis_->MotionIdGet() << "\n";
            printAllAxisMotionStatus("All axes immediately after joint-vector MoveRelative");

            waitForAllAxisMotionStart(
                "Joint-vector MultiAxis::MoveRelative",
                startingCommandPositions);

            for (int sample = 0; sample < 8; ++sample)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(MotionStatusSampleMs));
                printAllAxisProgressLine("Joint-vector live sample", sample + 1);
            }

            waitForMotionDone(MotionTimeoutMs);

            printActualPositions("Actual positions after joint-vector MultiAxis::MoveRelative step");
            printAllAxisMotionStatus("All axes after MotionDoneWait");
        }
    }
    catch (...)
    {
        if (errorLimitsTemporarilyChanged)
        {
            for (int index = 0; index < AxisCount; ++index)
            {
                axes_[index]->ErrorLimitActionSet(originalErrorLimitActions[index]);
                std::cout << "Restored Axis " << (index + 1)
                          << " ErrorLimitAction to " << actionName(originalErrorLimitActions[index]) << ".\n";
            }
        }

        throw;
    }

    if (errorLimitsTemporarilyChanged)
    {
        for (int index = 0; index < AxisCount; ++index)
        {
            axes_[index]->ErrorLimitActionSet(originalErrorLimitActions[index]);
            std::cout << "Restored Axis " << (index + 1)
                      << " ErrorLimitAction to " << actionName(originalErrorLimitActions[index]) << ".\n";
        }
    }

    std::cout << "Custom joint-vector MultiAxis MoveRelative diagnostic complete. Net commanded offsets are zero.\n";
}





void printRapidVectorDouble(const char* label, const RR::RapidVector<double>& values)
{
    std::cout << "  " << label << " [size=" << values.Size() << "]: ";
    std::cout << std::fixed << std::setprecision(9);

    for (size_t index = 0; index < values.Size(); ++index)
    {
        std::cout << values.At(index);
        if (index + 1 < values.Size())
        {
            std::cout << ", ";
        }
    }

    std::cout << "\n";
}

void printVector3d(const char* label, const RC::Vector3d& vector)
{
    std::cout << "  " << label
              << " X=" << std::fixed << std::setprecision(9) << vector.X
              << " Y=" << vector.Y
              << " Z=" << vector.Z
              << "\n";
}

void printQuaternion(const char* label, const RC::Quaternion& quaternion)
{
    std::cout << "  " << label
              << " W=" << std::fixed << std::setprecision(9) << quaternion.W
              << " X=" << quaternion.V.X
              << " Y=" << quaternion.V.Y
              << " Z=" << quaternion.V.Z
              << "\n";
}

void printPose(const char* label, const RC::Pose& pose)
{
    std::cout << "  " << label << ":\n";
    printVector3d("    Position", pose.Position);
    printQuaternion("    Quaternion", pose.Orientation);

    try
    {
        const RC::Vector3d eulerRadians = pose.Orientation.ToEuler();
        printVector3d("    EulerRadians", eulerRadians);
        std::cout << "  " << "    EulerDegrees"
                  << " R=" << std::fixed << std::setprecision(9) << (eulerRadians.X * 180.0 / 3.14159265358979323846)
                  << " P=" << (eulerRadians.Y * 180.0 / 3.14159265358979323846)
                  << " Y=" << (eulerRadians.Z * 180.0 / 3.14159265358979323846)
                  << "\n";
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "    ToEuler threw RapidCode error: " << error.text
                  << " (" << error.functionName << ")\n";
    }
    catch (...)
    {
        std::cout << "    ToEuler threw unknown exception.\n";
    }
}

void printRobotPosition(const char* label, const RC::RobotPosition& position)
{
    std::cout << "  " << label << ":\n";
    printPose("    Pose", position.Pose);
    printRapidVectorDouble("    FreeAxes", position.FreeAxes);
}

RR::RapidVector<double> makeJointRapidVector(const JointVector& values)
{
    RR::RapidVector<double> vector(values.size());

    for (size_t index = 0; index < values.size(); ++index)
    {
        vector.At(index) = values[index];
    }

    return vector;
}

void printFkForSyntheticVector(
    RC::Robot* robot,
    const char* label,
    const JointVector& jointValues)
{
    std::cout << "\n  Synthetic FK vector: " << label << "\n";

    try
    {
        const RR::RapidVector<double> joints = makeJointRapidVector(jointValues);
        printRapidVectorDouble("    Input joints", joints);

        const RC::Pose fkPose = robot->ForwardKinematics(joints);
        printPose("    ForwardKinematics Pose", fkPose);

        const RC::RobotPosition fkPosition = robot->ForwardKinematicsPosition(joints);
        printRobotPosition("    ForwardKinematicsPosition", fkPosition);

        const RR::RapidVector<double> ikFromFkPose = robot->InverseKinematics(fkPose);
        printRapidVectorDouble("    InverseKinematics(FK Pose)", ikFromFkPose);

        const RR::RapidVector<double> ikFromFkPosition = robot->InverseKinematics(fkPosition);
        printRapidVectorDouble("    InverseKinematics(FK Position)", ikFromFkPosition);
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "    Synthetic FK/IK RapidCode error: " << error.text
                  << " (" << error.functionName << ")\n";
    }
    catch (const std::exception& error)
    {
        std::cout << "    Synthetic FK/IK std::exception: " << error.what() << "\n";
    }
    catch (...)
    {
        std::cout << "    Synthetic FK/IK unknown exception.\n";
    }
}

void printSyntheticFkComparisonSet(RC::Robot* robot)
{
    static constexpr double testDelta = 0.01;

    JointVector zeros{};
    printFkForSyntheticVector(robot, "all zeros", zeros);

    for (int axis = 0; axis < Racer3BasicMotion::AxisCount; ++axis)
    {
        JointVector vector{};
        vector[axis] = testDelta;

        std::ostringstream label;
        label << "J" << (axis + 1) << " +" << testDelta;
        printFkForSyntheticVector(robot, label.str().c_str(), vector);
    }

    JointVector all{};
    all.fill(testDelta);
    printFkForSyntheticVector(robot, "all axes +0.01", all);
}


template <typename Callable>
void probeReadOnlyRobotCall(const char* label, Callable callable)
{
    try
    {
        auto result = callable();
        (void)result;
        std::cout << "  " << label << ": OK\n";
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "  " << label << ": RapidCode error: " << error.text
                  << " (" << error.functionName << ")\n";
    }
    catch (const std::exception& error)
    {
        std::cout << "  " << label << ": std::exception: " << error.what() << "\n";
    }
    catch (...)
    {
        std::cout << "  " << label << ": unknown exception.\n";
    }
}

void runReadOnlyRobotApiProbe(
    RR::MotionController* controller,
    RR::MultiAxis* multiAxis,
    const char* modelLabel,
    const char* probeLabel)
{
    std::cout << "\n" << probeLabel << ": numeric read-only Robot API probe using LinearModelBuilder(\""
              << modelLabel
              << "\")\n";

    RC::Robot* robot = nullptr;

    try
    {
        RC::LinearModelBuilder builder(modelLabel);
        addSixAxisXyzAbcLinearJoints(builder);
        const RC::KinematicModel& model = builder.ModelBuild();
        (void)model;

        robot = RC::Robot::RobotCreate(controller, multiAxis, &builder);
        if (!robot)
        {
            std::cout << "  RobotCreate returned null. Skipping numeric read-only API calls.\n";
            return;
        }

        std::cout << "  RobotCreate returned a non-null Robot pointer.\n";
        printErrorLog("  Robot(read-only numeric probe)", robot);

        try
        {
            const RR::RapidVector<double> jointsActual = robot->JointsActualPositionsGet();
            printRapidVectorDouble("JointsActualPositionsGet", jointsActual);
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  JointsActualPositionsGet RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }

        try
        {
            const RR::RapidVector<double> jointsCommand = robot->JointsCommandPositionsGet();
            printRapidVectorDouble("JointsCommandPositionsGet", jointsCommand);
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  JointsCommandPositionsGet RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }

        try
        {
            const RC::Pose actualPose = robot->ActualPoseGet();
            printPose("ActualPoseGet", actualPose);
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  ActualPoseGet RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }

        try
        {
            const RC::Pose commandPose = robot->CommandPoseGet();
            printPose("CommandPoseGet", commandPose);
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  CommandPoseGet RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }

        try
        {
            const RC::RobotPosition actualPosition = robot->ActualPositionGet();
            printRobotPosition("ActualPositionGet", actualPosition);
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  ActualPositionGet RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }

        try
        {
            const RC::RobotPosition commandPosition = robot->CommandPositionGet();
            printRobotPosition("CommandPositionGet", commandPosition);
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  CommandPositionGet RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }

        try
        {
            const RR::RapidVector<double> jointsActual = robot->JointsActualPositionsGet();
            const RC::Pose fkPose = robot->ForwardKinematics(jointsActual);
            printPose("ForwardKinematics(JointsActualPositionsGet())", fkPose);

            const RC::RobotPosition fkPosition = robot->ForwardKinematicsPosition(jointsActual);
            printRobotPosition("ForwardKinematicsPosition(JointsActualPositionsGet())", fkPosition);
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  ForwardKinematics actual-joints RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }

        try
        {
            const RC::Pose actualPose = robot->ActualPoseGet();
            const RR::RapidVector<double> ik = robot->InverseKinematics(actualPose);
            printRapidVectorDouble("InverseKinematics(ActualPoseGet())", ik);
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  InverseKinematics(ActualPoseGet()) RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }

        try
        {
            const RC::RobotPosition actualPosition = robot->ActualPositionGet();
            const RR::RapidVector<double> ik = robot->InverseKinematics(actualPosition);
            printRapidVectorDouble("InverseKinematics(ActualPositionGet())", ik);
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  InverseKinematics(ActualPositionGet()) RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }

        std::cout << "\n  Synthetic FK/IK comparison set for " << modelLabel << ":\n";
        std::cout << "  Test delta is 0.01 in robot joint units. For a purely linear XYZABC model, J1/J2/J3 should map directly to X/Y/Z and J4/J5/J6 to orientation axes.\n";
        printSyntheticFkComparisonSet(robot);
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "  " << probeLabel << " setup RapidCode error: " << error.text << "\n";
        std::cout << "  Function: " << error.functionName << "\n";
    }
    catch (const std::exception& error)
    {
        std::cout << "  " << probeLabel << " setup std::exception: " << error.what() << "\n";
    }
    catch (...)
    {
        std::cout << "  " << probeLabel << " setup unknown exception.\n";
    }

    if (robot)
    {
        try
        {
            RC::Robot::RobotDelete(controller, robot);
            std::cout << "  RobotDelete completed.\n";
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  RobotDelete RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }
        catch (...)
        {
            std::cout << "  RobotDelete unknown exception.\n";
        }
    }
}


void Racer3BasicMotion::runRobotPoseProbe()
{
    std::cout << "Starting no-motion Cartesian Robot pose/FK API probe.\n";
    std::cout << "No amp enable and no motion commands will be sent in this mode.\n";
    std::cout << "Faults are cleared so read-only pose/joint APIs can run from a non-error state.\n";

    if (!controller_)
    {
        throw std::runtime_error("MotionController object is not initialized.");
    }

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    setRobotProbeAxisLabels(axes_);

    std::cout << "Aborting and clearing faults through MultiAxis 6 for read-only pose API access...\n";
    multiAxis_->Abort();
    multiAxis_->ClearFaults();
    std::this_thread::sleep_for(std::chrono::milliseconds(FaultClearSettleMs));

    printMotionProgressLine("Robot pose probe pre-check", 0);

    runReadOnlyRobotApiProbe(
        controller_,
        multiAxis_,
        "RSI_Racer3",
        "Pose Probe 1");

    runReadOnlyRobotApiProbe(
        controller_,
        multiAxis_,
        "RSI_XYZABC_Millimeters",
        "Pose Probe 2");

    std::cout << "\nController and MultiAxis error logs after robot pose/FK probe:\n";
    printErrorLog("MotionController", controller_);
    printErrorLog("MultiAxis 6", multiAxis_);

    std::cout << "Robot pose/FK numeric probe complete.\n";
    std::cout << "Compare the RSI_Racer3 synthetic FK outputs against RSI_XYZABC_Millimeters. If they match exactly, the configured Racer3 builder is behaving like a linear XYZABC mapping rather than true articulated arm kinematics.\n";
}


void Racer3BasicMotion::runRobotModelProbe()
{
    std::cout << "Starting no-motion Cartesian Robot model probe.\n";
    std::cout << "No amp enable, no fault clear, and no motion commands will be sent in this mode.\n";

    if (!controller_)
    {
        throw std::runtime_error("MotionController object is not initialized.");
    }

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    std::cout << "MultiAxis 6 should contain Axis 1 through Axis 6 from configureAxes().\n";
    printMotionProgressLine("Robot probe pre-check", 0);

    setRobotProbeAxisLabels(axes_);

    const char* const racer3Label = "RSI_Racer3";
    const char* const xyzabcLabel = "RSI_XYZABC_Millimeters";

    std::cout << "\nProbe 1: LinearModelBuilder(\"" << racer3Label << "\") with no JointAdd calls.\n";
    std::cout << "This is expected to show whether the label alone configures a model.\n";
    try
    {
        RC::LinearModelBuilder builder(racer3Label);

        std::cout << "  LinearModelBuilder constructed.\n";
        const RC::KinematicModel& model = builder.ModelBuild();
        (void)model;
        std::cout << "  ModelBuild returned a KinematicModel reference.\n";

        RC::Robot* robot = RC::Robot::RobotCreate(controller_, multiAxis_, &builder);
        if (!robot)
        {
            std::cout << "  RobotCreate returned null.\n";
        }
        else
        {
            std::cout << "  RobotCreate returned a non-null Robot pointer.\n";
            printErrorLog("  Robot(unconfigured builder)", robot);
            RC::Robot::RobotDelete(controller_, robot);
            std::cout << "  RobotDelete completed for unconfigured builder Robot.\n";
        }
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "  Probe 1 RapidCode error: " << error.text << "\n";
        std::cout << "  Function: " << error.functionName << "\n";
    }
    catch (const std::exception& error)
    {
        std::cout << "  Probe 1 std::exception: " << error.what() << "\n";
    }
    catch (...)
    {
        std::cout << "  Probe 1 unknown exception.\n";
    }

    probeConfiguredLinearBuilder(
        controller_,
        multiAxis_,
        racer3Label,
        "Probe 2");

    probeConfiguredLinearBuilder(
        controller_,
        multiAxis_,
        xyzabcLabel,
        "Probe 3");

    std::cout << "\nProbe 4: RobotCreate(controller, multiAxis, \"" << racer3Label << "\") direct modelIdentifier path.\n";
    try
    {
        RC::Robot* robot = RC::Robot::RobotCreate(controller_, multiAxis_, racer3Label);
        if (!robot)
        {
            std::cout << "  RobotCreate returned null.\n";
        }
        else
        {
            std::cout << "  RobotCreate returned a non-null Robot pointer.\n";
            printErrorLog("  Robot(modelIdentifier)", robot);
            RC::Robot::RobotDelete(controller_, robot);
            std::cout << "  RobotDelete completed for identifier-created Robot.\n";
        }
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "  Probe 4 RapidCode error: " << error.text << "\n";
        std::cout << "  Function: " << error.functionName << "\n";
    }
    catch (const std::exception& error)
    {
        std::cout << "  Probe 4 std::exception: " << error.what() << "\n";
    }
    catch (...)
    {
        std::cout << "  Probe 4 unknown exception.\n";
    }

    std::cout << "\nController and MultiAxis error logs after robot model probe:\n";
    printErrorLog("MotionController", controller_);
    printErrorLog("MultiAxis 6", multiAxis_);

    std::cout << "Robot model probe complete. Review Robot error log counts above.\n";
    std::cout << "Interpretation:\n";
    std::cout << "  - Count=0 on Probe 2 means RSI_Racer3 can create a configured Robot when Axis labels and JointAdd mappings match.\n";
    std::cout << "  - Count=0 on Probe 3 means the known generic XYZABC linear model path works.\n";
    std::cout << "  - Even if Probe 2 succeeds, this does not prove true articulated Racer3 FK/IK. It may be a linear Cartesian mapping.\n";
}



void Racer3BasicMotion::runKinematicsDryRun()
{
    std::cout << "Starting no-motion custom Racer3 kinematics dry-run.\n";
    std::cout << "No amp enable and no motion commands will be sent in this mode.\n";
    std::cout << "This is a FK scaffold from the old rapidrobot OpenRAVE model, not a validated production IK planner yet.\n";

    if (!controller_)
    {
        throw std::runtime_error("MotionController object is not initialized.");
    }

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    std::cout << "\nOpenRAVE/RapidRobot model inputs used by this scaffold:\n";
    std::cout << "  Tool point at zero pose: [0, 0, 1.012] meters.\n";
    std::cout << "  Joint anchors and axes are hardcoded from racer3.kinbody.xml.\n";
    std::cout << "  RapidRobot saved AbsoluteSingleTurn home offset is also printed for comparison.\n";
    std::cout << "  RMP user units are still 1.0 revolution per joint; radians = user_units * 2*pi.\n";

    printCartesianVector("Requested Cartesian delta", RequestedCartesianVector);

    JointVector actualUserUnits{};
    JointVector commandUserUnits{};

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            throw std::runtime_error("Axis " + std::to_string(index + 1) + " is not initialized.");
        }

        actualUserUnits[index] = axes_[index]->ActualPositionGet();
        commandUserUnits[index] = axes_[index]->CommandPositionGet();
    }

    std::cout << "\nCurrent RMP joint state after software-zero/configuration:\n";
    std::cout << "  Actual user units/revolutions: ";
    printJointVector(actualUserUnits);
    std::cout << "  Command user units/revolutions: ";
    printJointVector(commandUserUnits);

    JointVector actualRadians{};
    JointVector commandRadians{};
    JointVector openRaveHomeAdjustedActualRadians{};

    for (int index = 0; index < AxisCount; ++index)
    {
        actualRadians[index] = actualUserUnits[index] * RevolutionsToRadians;
        commandRadians[index] = commandUserUnits[index] * RevolutionsToRadians;
        openRaveHomeAdjustedActualRadians[index] =
            actualRadians[index] + RapidRobotAbsoluteSingleTurnHomeRadians[index];
    }

    printOpenRaveFkReport(
        "FK from current RMP actual joints interpreted relative to the current software zero",
        actualRadians);

    printOpenRaveFkReport(
        "FK from current RMP command joints interpreted relative to the current software zero",
        commandRadians);

    printOpenRaveFkReport(
        "FK from current actual joints plus rapidrobot AbsoluteSingleTurn home offset",
        openRaveHomeAdjustedActualRadians);

    const FkMat4 currentTransform = openRaveRacer3ForwardKinematics(actualRadians);
    const FkVec3 currentPosition = positionFromTransform(currentTransform);
    const FkVec3 currentRpy = rpyFromTransform(currentTransform);

    const FkVec3 targetPosition = {
        currentPosition.x + RequestedCartesianVector[0],
        currentPosition.y + RequestedCartesianVector[1],
        currentPosition.z + RequestedCartesianVector[2]
    };

    const FkVec3 targetRpy = {
        currentRpy.x + RequestedCartesianVector[3],
        currentRpy.y + RequestedCartesianVector[4],
        currentRpy.z + RequestedCartesianVector[5]
    };

    std::cout << "\nCartesian target preview from requested delta:\n";
    printFkVec3("  Current TCP position", currentPosition, "meters");
    printFkVec3("  Current TCP RPY", currentRpy, "radians");
    printFkVec3("  Target TCP position", targetPosition, "meters");
    printFkVec3("  Target TCP RPY", targetRpy, "radians");

    CartesianVector targetPoseVector = {
        targetPosition.x,
        targetPosition.y,
        targetPosition.z,
        targetRpy.x,
        targetRpy.y,
        targetRpy.z
    };

    printMultiSeedIkDryRunReport(
        "Multi-seed numerical IK dry-run from current software-zero joint neighborhood",
        actualRadians,
        targetPoseVector);

    printMultiSeedIkDryRunReport(
        "Multi-seed numerical IK dry-run from rapidrobot AbsoluteSingleTurn home-offset neighborhood",
        openRaveHomeAdjustedActualRadians,
        targetPoseVector);

    std::cout << "\nKinematics dry-run result:\n";
    std::cout << "  FK scaffold: available.\n";
    std::cout << "  Multi-seed numerical IK dry-run with dry-run validation gates: available.\n";
    std::cout << "  Motion execution: intentionally disabled.\n";
    std::cout << "  Next step: use only candidates that pass residual and joint-delta gates, then add a guarded Cartesian dry-run-to-motion path.\n";

    std::cout << "\nSafety note:\n";
    std::cout << "  The IK result is a candidate only. It is not commanded in this patch.\n";
    std::cout << "  Absolute FK/IK pose is only meaningful after we confirm the physical robot zero/home convention matches the OpenRAVE/RapidRobot model convention.\n";
    std::cout << "  Until then, use this as a model-development diagnostic, not as a live Cartesian motion command.\n";

    std::cout << "\nController and MultiAxis error logs after kinematics dry-run:\n";
    printErrorLog("MotionController", controller_);
    printErrorLog("MultiAxis 6", multiAxis_);
}


void Racer3BasicMotion::runCartesianVectorMotion()
{
    std::cout << "Starting guarded segmented Cartesian-vector IK mode.\n";
    std::cout << "This mode computes custom OpenRAVE IK candidates for one or more absolute Cartesian waypoints.\n";
    std::cout << "A large request can pass by being split into smaller validated absolute waypoints instead of raising the joint-delta gate.\n";
    std::cout << "IK residual mode: " << ikResidualModeName() << "\n";
    if (PositionOnlyIkEnabled)
    {
        std::cout << "Position-only IK is active: solve/gates use XYZ only and allow wrist orientation to change naturally.\n";
    }
    std::cout << "Segmented execution logging mode: " << compactSegmentedExecutionName() << "\n";
    std::cout << "Segmented motion execution mode: " << segmentedMotionExecutionModeName() << "\n";
    if (CompactSegmentedExecutionEnabled)
    {
        std::cout << "CompactMotion is active: per-segment live samples and large status dumps are skipped to reduce stop-to-stop delay.\n";
        if (!AppendSegmentedExecutionEnabled && !TrajectorySegmentedExecutionEnabled)
        {
            std::cout << "This is still sequential MoveRelative execution, not true controller-side blending.\n";
        }
    }
    if (AppendSegmentedExecutionEnabled)
    {
        std::cout << "AppendMotion is active: outbound and return segment commands will be queued with APPEND and one MotionDoneWait per phase.\n";
        std::cout << "NO_WAIT remains off in this first guarded path so RapidCode still acknowledges queued command errors.\n";
    }
    if (TrajectorySegmentedExecutionEnabled)
    {
        std::cout << "TrajectoryMotion is active: validated joint endpoints will be streamed as MultiAxis::MovePVT waypoints.\n";
        std::cout << "This keeps the same IK gates, but asks RapidCode for continuous waypoint velocity instead of appended point-to-point profiles.\n";
    }

    if (EndpointOnlyMotionEnabled)
    {
        std::cout << "EndpointOnly is active: solve one final XYZ target, then run a smooth joint-space PVT move.\n";
        std::cout << "This does not guarantee a straight Cartesian TCP line, but it avoids segmented IK branch changes.\n";
    }

    if (SegmentGoalMotionEnabled)
    {
        std::cout << "SegmentGoal is active: use segmented IK only to find the final joint target, then run one smooth joint-space PVT move.\n";
        std::cout << "This should be smoother than streaming all segmented IK waypoints, but it does not guarantee a straight Cartesian TCP line.\n";
    }

    if (!controller_)
    {
        throw std::runtime_error("MotionController object is not initialized.");
    }

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    printCartesianVector("Requested Cartesian delta", RequestedCartesianVector);

    JointVector actualUserUnits{};

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            throw std::runtime_error("Axis " + std::to_string(index + 1) + " is not initialized.");
        }

        actualUserUnits[index] = axes_[index]->ActualPositionGet();
    }

    JointVector actualRadians{};

    for (int index = 0; index < AxisCount; ++index)
    {
        actualRadians[index] = actualUserUnits[index] * RevolutionsToRadians;
    }

    const FkMat4 currentTransform = openRaveRacer3ForwardKinematics(actualRadians);
    const FkVec3 currentPosition = positionFromTransform(currentTransform);
    const FkVec3 currentRpy = rpyFromTransform(currentTransform);

    const FkVec3 targetPosition = {
        currentPosition.x + RequestedCartesianVector[0],
        currentPosition.y + RequestedCartesianVector[1],
        currentPosition.z + RequestedCartesianVector[2]
    };

    const FkVec3 targetRpy = {
        currentRpy.x + RequestedCartesianVector[3],
        currentRpy.y + RequestedCartesianVector[4],
        currentRpy.z + RequestedCartesianVector[5]
    };

    std::cout << "\nCurrent and final target pose preview:\n";
    printJointDegreesFromRadians("  Current joint degrees", actualRadians);
    printFkVec3("  Current TCP position", currentPosition, "meters");
    printFkVec3("  Current TCP RPY", currentRpy, "radians");
    printFkVec3("  Final target TCP position", targetPosition, "meters");
    printFkVec3("  Final target TCP RPY", targetRpy, "radians");

    if (EndpointOnlyMotionEnabled || SegmentGoalMotionEnabled)
    {
        const bool usingSegmentGoal = SegmentGoalMotionEnabled;

        std::cout << "\n"
                  << (usingSegmentGoal ? "Segment-goal position target mode" : "Endpoint-only position target mode")
                  << "\n";

        if (usingSegmentGoal)
        {
            std::cout << "Purpose: use the segmented Cartesian IK planner only to discover a reachable final joint target, then move to that final target smoothly in joint space.\n";
            std::cout << "This avoids streaming the segmented IK waypoint path that caused visible shake.\n";
        }
        else
        {
            std::cout << "Purpose: reach the final XYZ point smoothly in joint space; TCP path is not forced to be a straight Cartesian line.\n";
        }

        JointVector targetJointRadians{};
        IkDryRunResult finalValidation{};
        std::string targetSourceName;
        int segmentGoalCount = 0;

        if (usingSegmentGoal)
        {
            std::cout << "\nBuilding absolute-waypoint segmented IK plan to discover final reachable joint goal...\n";
            const CartesianSegmentPlan discoveryPlan =
                buildSegmentedCartesianPlan(actualRadians, RequestedCartesianVector);

            std::cout << "Segment-goal discovery summary:\n";
            std::cout << "  Accepted: " << (discoveryPlan.accepted ? "YES" : "NO") << "\n";
            std::cout << "  Segment count evaluated/selected: " << discoveryPlan.segmentCount << "\n";

            if (!discoveryPlan.accepted)
            {
                std::cout << "  Rejection reason: " << discoveryPlan.rejectionReason << "\n";
                std::cout << "Segment-goal discovery failed. No motion commanded.\n";
                return;
            }

            if (discoveryPlan.segments.empty())
            {
                std::cout << "Segment-goal discovery produced no segments. No motion commanded.\n";
                return;
            }

            const CartesianSegmentCandidate& finalSegment = discoveryPlan.segments.back();
            targetJointRadians = finalSegment.candidate.result.solutionRadians;
            segmentGoalCount = discoveryPlan.segmentCount;
            targetSourceName = "segmented IK final joint goal";

            const CartesianVector globalTargetPose =
                targetPoseVectorForCartesianDelta(actualRadians, RequestedCartesianVector);
            const CartesianVector finalPose = poseVectorFromJoints(targetJointRadians);

            finalValidation = finalSegment.candidate.result;
            finalValidation.solutionRadians = targetJointRadians;
            finalValidation.deltaRadians = subtractJointVectors(targetJointRadians, actualRadians);
            finalValidation.residual = subtractPoseVectorWrapped(globalTargetPose, finalPose);
            finalValidation.residualNorm = residualNormForIkMode(finalValidation.residual);
            finalValidation.maxResidualComponent = maxAbsResidualComponentForIkMode(finalValidation.residual);
            finalValidation.hitJointLimit =
                finalSegment.candidate.result.hitJointLimit || isOutsideJointLimits(targetJointRadians);

            std::cout << "  Final segment source seed: " << finalSegment.candidate.seedName << "\n";
            std::cout << "  Final segment solver converged flag: "
                      << boolText(finalSegment.candidate.result.converged)
                      << "\n";
            std::cout << "  Final global XYZ residual is recomputed against the original requested final target. This should now match the absolute-waypoint final gate.\n";
        }
        else
        {
            const IkBestCandidate endpointCandidate =
                solveBestCandidateForCartesianDelta(actualRadians, RequestedCartesianVector);

            if (!endpointCandidate.found)
            {
                std::cout << "Endpoint-only IK failed: no IK candidate was evaluated. No motion commanded.\n";
                return;
            }

            targetJointRadians = endpointCandidate.result.solutionRadians;
            finalValidation = endpointCandidate.result;
            finalValidation.deltaRadians = subtractJointVectors(targetJointRadians, actualRadians);
            targetSourceName = endpointCandidate.seedName;
        }

        const JointVector endpointDeltaRadians =
            subtractJointVectors(targetJointRadians, actualRadians);
        const JointVector endpointDeltaUserUnits =
            radiansToUserUnits(endpointDeltaRadians);
        const double endpointMaxDeltaDegrees =
            maxAbsJointDeltaDegrees(endpointDeltaRadians);
        const bool endpointResidualOk =
            residualAcceptedForDryRunCandidate(finalValidation);
        const bool endpointJointLimitOk =
            !finalValidation.hitJointLimit;
        const bool endpointJointDeltaOk =
            endpointMaxDeltaDegrees <= EndpointMaxJointDeltaDegreesAccept;
        const bool endpointAccepted =
            endpointResidualOk && endpointJointLimitOk && endpointJointDeltaOk;

        std::cout << "\n"
                  << (usingSegmentGoal ? "Segment-goal smooth endpoint summary" : "Endpoint-only IK summary")
                  << "\n";
        std::cout << "  Target source: " << targetSourceName << "\n";
        if (usingSegmentGoal)
        {
            std::cout << "  Segment-goal discovery segments: " << segmentGoalCount << "\n";
        }
        std::cout << "  Hit joint limit: " << boolText(finalValidation.hitJointLimit) << "\n";
        std::cout << "  Gated XYZ residual norm: "
                  << std::fixed << std::setprecision(9)
                  << finalValidation.residualNorm << " m\n";
        std::cout << "  Gated XYZ max residual component: "
                  << finalValidation.maxResidualComponent << " m\n";
        std::cout << "  Full pose residual norm including RPY: "
                  << residualNorm(finalValidation.residual) << "\n";
        std::cout << "  Max endpoint joint delta: "
                  << endpointMaxDeltaDegrees << " degrees\n";
        std::cout << "  Endpoint joint-delta gate <= "
                  << EndpointMaxJointDeltaDegreesAccept
                  << " degrees: "
                  << (endpointJointDeltaOk ? "PASS" : "FAIL")
                  << "\n";
        if (endpointMaxDeltaDegrees > EndpointWarnJointDeltaDegrees)
        {
            std::cout << "  WARNING: endpoint joint delta is above "
                      << EndpointWarnJointDeltaDegrees
                      << " degrees. Review the pose before live motion.\n";
        }
        std::cout << "  Residual gate: " << (endpointResidualOk ? "PASS" : "FAIL") << "\n";
        std::cout << "  Joint-limit gate: " << (endpointJointLimitOk ? "PASS" : "FAIL") << "\n";
        std::cout << "  Smooth endpoint accepted: " << (endpointAccepted ? "YES" : "NO") << "\n";

        printCartesianResidual("  Final residual", finalValidation.residual);
        printJointDegreesFromRadians("  Current joint degrees", actualRadians);
        printJointDegreesFromRadians("  Smooth endpoint target joint degrees", targetJointRadians);
        printJointDegreesFromRadians("  Smooth endpoint command delta degrees", endpointDeltaRadians);
        printJointUserUnitsFromRadians("  Smooth endpoint command delta user units/revolutions", endpointDeltaRadians);
        printJointLimitReport(targetJointRadians);
        printOpenRaveFkReport(
            usingSegmentGoal ? "  FK verification for segment-goal target" : "  FK verification for endpoint-only target",
            targetJointRadians);

        if (!endpointAccepted)
        {
            std::cout << "\nSmooth endpoint candidate rejected by validation gates. No motion commanded.\n";
            return;
        }

        if (!CartesianVectorMotionConfirmed)
        {
            std::cout << "\nNo --confirm-motion was supplied. Smooth endpoint dry run only.\n";
            std::cout << "Live behavior would stream one smooth PVT outbound move to the final joint target, wait once, then stream one smooth PVT return to software zero.\n";
            std::cout << "No amps were enabled and no motion was commanded.\n";
            return;
        }

        std::cout << "\n--confirm-motion supplied and smooth endpoint gates passed.\n";
        std::cout << "Executing one smooth joint-space PVT move to the endpoint, then one smooth PVT return to software zero.\n";

        clearFaults();

        printActualPositions("Actual positions after smooth endpoint validation, before amp enable");
        printDiagnosticSnapshot("After smooth endpoint validation and clear faults, before amp enable");

        enableAmplifiers();

        printActualPositions("Actual positions after amp enable for smooth endpoint motion");
        printDiagnosticSnapshot("After amp enable for smooth endpoint motion");

        isolateAllAxesForAllMotion();

        std::array<RR::RSIAction, AxisCount> originalErrorLimitActions{};
        bool errorLimitsTemporarilyChanged = false;

        if (TemporarilyDisableAxis6ErrorLimitForTinyMotion)
        {
            std::cout << "Temporarily setting all axes position ErrorLimitAction to RSIActionNONE for smooth endpoint motion.\n";
            std::cout << "  Amp fault and hardware limit actions are not changed.\n";

            for (int index = 0; index < AxisCount; ++index)
            {
                originalErrorLimitActions[index] = axes_[index]->ErrorLimitActionGet();
                std::cout << "  Original Axis " << (index + 1)
                          << " ErrorLimitAction: "
                          << actionName(originalErrorLimitActions[index])
                          << "\n";
                axes_[index]->ErrorLimitActionSet(RR::RSIAction::RSIActionNONE);
            }

            errorLimitsTemporarilyChanged = true;
        }

        auto readAllAxisCommandPositions = [&]() {
            JointVector commandPositions{};

            for (int index = 0; index < AxisCount; ++index)
            {
                commandPositions[index] = axes_[index]->CommandPositionGet();
            }

            return commandPositions;
        };

        auto commandSmoothEndpointPvtPhase = [&](const char* phaseLabel, const JointVector& targetCommandPosition) {
            std::cout << "\n=== Smooth endpoint PVT "
                      << phaseLabel
                      << " phase ===\n";

            const JointVector startingCommandPositions = readAllAxisCommandPositions();
            const double samplePeriodSeconds = samplePeriodSecondsFromController(controller_);
            const JointTrajectoryBlock trajectory =
                makeSmoothEndpointPvtBlock(
                    startingCommandPositions,
                    targetCommandPosition,
                    samplePeriodSeconds);
            const std::vector<double> flatPositions =
                flattenJointTrajectoryPoints(trajectory.positions);
            const std::vector<double> flatVelocities =
                flattenJointTrajectoryPoints(trajectory.velocities);

            std::cout << "Streaming "
                      << trajectory.positions.size()
                      << " smooth endpoint MultiAxis::MovePVT waypoint(s), then waiting once for phase completion.\n";
            std::cout << "  Controller sample period used for time rounding: "
                      << std::fixed << std::setprecision(6)
                      << samplePeriodSeconds
                      << " sec.\n";
            std::cout << "  Total planned phase time: "
                      << trajectory.totalSeconds
                      << " sec; point dt min/max: "
                      << minTrajectoryPointSeconds(trajectory)
                      << " / "
                      << maxTrajectoryPointSeconds(trajectory)
                      << " sec.\n";
            std::cout << "  Max specified waypoint velocity: "
                      << maxAbsTrajectoryVelocity(trajectory)
                      << " user-units/sec = "
                      << toDegrees(maxAbsTrajectoryVelocity(trajectory))
                      << " deg/sec.\n";

            const std::string multiAxisContext =
                std::string("before smooth endpoint PVT ") + phaseLabel + " phase";
            configureMultiAxisMotionAttributes(multiAxisContext.c_str());
            multiAxis_->MotionAttributeMaskOffSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskAPPEND);
            multiAxis_->MotionAttributeMaskOffSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskNO_WAIT);

            for (int index = 0; index < AxisCount; ++index)
            {
                const std::string axisContext =
                    std::string("before smooth endpoint PVT ") + phaseLabel + " phase";
                configureAxisMotionAttributes(index, axisContext.c_str());
            }

            if (!CompactSegmentedExecutionEnabled)
            {
                printAllAxisMotionStatus("All axes before smooth endpoint PVT phase");
            }

            const uint16_t motionIdBefore = multiAxis_->MotionIdGet();

            multiAxis_->MovePVT(
                flatPositions.data(),
                flatVelocities.data(),
                trajectory.times.data(),
                static_cast<int32_t>(trajectory.positions.size()),
                TrajectoryPvtEmptyCount,
                false,
                true);

            std::cout << "  MotionId "
                      << motionIdBefore
                      << " -> "
                      << multiAxis_->MotionIdGet()
                      << " (smooth endpoint PVT stream).\n";

            const std::string startLabel =
                std::string("Smooth endpoint PVT ") + phaseLabel + " MultiAxis::MovePVT";
            waitForAllAxisMotionStart(startLabel.c_str(), startingCommandPositions);

            const int phaseTimeoutMilliseconds =
                trajectoryBlockMotionTimeoutMs(trajectory);
            std::cout << "Waiting once for smooth endpoint PVT "
                      << phaseLabel
                      << " phase completion. Timeout="
                      << phaseTimeoutMilliseconds
                      << " ms.\n";
            waitForMotionDone(phaseTimeoutMilliseconds);

            const std::string actualLabel =
                std::string("Actual positions after smooth endpoint PVT ") + phaseLabel + " phase";
            printActualPositions(actualLabel.c_str());

            const std::string resetContext =
                std::string("after smooth endpoint PVT ") + phaseLabel + " phase";
            configureMultiAxisMotionAttributes(resetContext.c_str());
        };

        try
        {
            const JointVector outboundStartCommandPositions = readAllAxisCommandPositions();
            JointVector endpointTargetCommandPositions{};

            for (int index = 0; index < AxisCount; ++index)
            {
                endpointTargetCommandPositions[index] =
                    outboundStartCommandPositions[index] +
                    endpointDeltaUserUnits[index];
            }

            JointVector softwareZeroCommandPositions{};
            softwareZeroCommandPositions.fill(0.0);

            commandSmoothEndpointPvtPhase("outbound", endpointTargetCommandPositions);
            commandSmoothEndpointPvtPhase("return", softwareZeroCommandPositions);
        }
        catch (...)
        {
            if (errorLimitsTemporarilyChanged)
            {
                for (int index = 0; index < AxisCount; ++index)
                {
                    axes_[index]->ErrorLimitActionSet(originalErrorLimitActions[index]);
                    std::cout << "Restored Axis "
                              << (index + 1)
                              << " ErrorLimitAction to "
                              << actionName(originalErrorLimitActions[index])
                              << " after exception.\n";
                }
            }

            throw;
        }

        if (errorLimitsTemporarilyChanged)
        {
            for (int index = 0; index < AxisCount; ++index)
            {
                axes_[index]->ErrorLimitActionSet(originalErrorLimitActions[index]);
                std::cout << "Restored Axis "
                          << (index + 1)
                          << " ErrorLimitAction to "
                          << actionName(originalErrorLimitActions[index])
                          << ".\n";
            }
        }

        std::cout << "Smooth endpoint PVT execution complete. Net commanded offsets are zero.\n";
        printReturnToZeroReport("Return-to-zero check after smooth endpoint Cartesian-vector motion", true);
        printActualPositions("Actual positions before smooth endpoint shutdown");
        printDiagnosticSnapshot("Before smooth endpoint Cartesian-vector shutdown");

        disableAmplifiers();

        std::cout << "Guarded smooth endpoint Cartesian-vector motion complete.\n";
        return;
    }

    const CartesianSegmentPlan segmentPlan =
        buildSegmentedCartesianPlan(actualRadians, RequestedCartesianVector);

    printCartesianSegmentPlanSummary(segmentPlan);

    if (!segmentPlan.accepted)
    {
        std::cout << "\nSegmented Cartesian-vector candidate rejected by validation gates. No motion commanded.\n";
        return;
    }

    if (segmentPlan.segmentCount > 1)
    {
        std::cout << "\nDirect one-shot motion would exceed the guarded joint-delta limit, so this request will use "
                  << segmentPlan.segmentCount
                  << " validated Cartesian segment(s).\n";
    }
    else
    {
        std::cout << "\nOne validated Cartesian segment is sufficient for this request.\n";
    }

    if (!CartesianVectorMotionConfirmed)
    {
        std::cout << "\nNo --confirm-motion was supplied. This was a guarded segmented Cartesian-vector dry run only.\n";
        if (AppendSegmentedExecutionEnabled)
        {
            std::cout << "AppendMotion live preview: queue "
                      << segmentPlan.segmentCount
                      << " outbound MoveRelative command(s), wait once, then queue "
                      << segmentPlan.segmentCount
                      << " return command(s), wait once.\n";
            std::cout << "APPEND would be off for the first command in each phase and on for the remaining command(s). NO_WAIT stays off.\n";
        }
        if (TrajectorySegmentedExecutionEnabled)
        {
            std::cout << "TrajectoryMotion live preview: stream "
                      << segmentPlan.segmentCount
                      << " outbound PVT waypoint(s), wait once, then stream "
                      << segmentPlan.segmentCount
                      << " return PVT waypoint(s), wait once.\n";
            std::cout << "PVT waypoints use the same validated segment endpoints with per-waypoint velocities chosen from neighboring segment slopes.\n";
        }
        std::cout << "No amps were enabled and no motion was commanded.\n";
        std::cout << "To execute the accepted segmented plan, rerun this mode with --confirm-motion after reviewing the segment deltas.\n";
        return;
    }

    std::cout << "\n--confirm-motion supplied and all segment gates passed.\n";
    std::cout << "Executing the segmented Cartesian IK plan through the MultiAxis joint-vector path.\n";
    if (AppendSegmentedExecutionEnabled)
    {
        std::cout << "Experimental append execution will queue outbound segments first, wait once, then queue return segments and wait once.\n";
    }
    else if (TrajectorySegmentedExecutionEnabled)
    {
        std::cout << "Experimental trajectory execution will stream outbound PVT waypoints first, wait once, then stream return PVT waypoints and wait once.\n";
    }
    else
    {
        std::cout << "The robot will execute each outbound segment, then execute the reverse segment sequence to return to software zero.\n";
    }

    clearFaults();

    printActualPositions("Actual positions after segmented Cartesian validation, before amp enable");
    printDiagnosticSnapshot("After segmented Cartesian validation and clear faults, before amp enable");

    enableAmplifiers();

    printActualPositions("Actual positions after amp enable for segmented Cartesian-vector motion");
    printDiagnosticSnapshot("After amp enable for segmented Cartesian-vector motion");

    const std::vector<JointVector> relativeSequence =
        makeOutAndBackSequenceFromSegmentPlan(segmentPlan);

    std::cout << "\nStarting segmented Cartesian-vector MultiAxis execution.\n";
    std::cout << "Sequence contains " << relativeSequence.size()
              << " joint-vector step(s): "
              << segmentPlan.segmentCount
              << " outbound and "
              << segmentPlan.segmentCount
              << " return.\n";

    if (relativeSequence.empty())
    {
        throw std::runtime_error("Segmented Cartesian sequence is empty.");
    }

    isolateAllAxesForAllMotion();

    std::array<RR::RSIAction, AxisCount> originalErrorLimitActions{};
    bool errorLimitsTemporarilyChanged = false;

    if (TemporarilyDisableAxis6ErrorLimitForTinyMotion)
    {
        std::cout << "Temporarily setting all axes position ErrorLimitAction to RSIActionNONE for segmented Cartesian-vector motion.\n";
        std::cout << "  Amp fault and hardware limit actions are not changed.\n";

        for (int index = 0; index < AxisCount; ++index)
        {
            originalErrorLimitActions[index] = axes_[index]->ErrorLimitActionGet();
            std::cout << "  Original Axis " << (index + 1)
                      << " ErrorLimitAction: " << actionName(originalErrorLimitActions[index]) << "\n";
            axes_[index]->ErrorLimitActionSet(RR::RSIAction::RSIActionNONE);
        }

        errorLimitsTemporarilyChanged = true;
    }

    const JointVector velocity = makeAllAxesVector(MotionVelocity);
    const JointVector acceleration = makeAllAxesVector(MotionAcceleration);
    const JointVector deceleration = makeAllAxesVector(MotionDeceleration);
    const JointVector jerk = makeAllAxesVector(MotionJerkPercent);

    try
    {
        if (TrajectorySegmentedExecutionEnabled)
        {
            const std::vector<JointVector> outboundSequence =
                makeOutboundSequenceFromSegmentPlan(segmentPlan);
            const std::vector<JointVector> returnSequence =
                makeReturnSequenceFromSegmentPlan(segmentPlan);

            if (outboundSequence.empty() || returnSequence.empty())
            {
                throw std::runtime_error("Trajectory segmented Cartesian sequence is empty.");
            }

            const double samplePeriodSeconds = samplePeriodSecondsFromController(controller_);

            auto readAllAxisCommandPositions = [&]() {
                JointVector commandPositions{};

                for (int index = 0; index < AxisCount; ++index)
                {
                    commandPositions[index] = axes_[index]->CommandPositionGet();
                }

                return commandPositions;
            };

            auto commandTrajectoryPhase = [&](const char* phaseLabel, const std::vector<JointVector>& phaseSequence) {
                std::cout << "\n=== PVT trajectory Cartesian-vector "
                          << phaseLabel
                          << " phase ===\n";

                const JointVector startingCommandPositions = readAllAxisCommandPositions();
                const JointTrajectoryBlock trajectory =
                    makePvtTrajectoryBlock(startingCommandPositions, phaseSequence, samplePeriodSeconds);
                const std::vector<double> flatPositions =
                    flattenJointTrajectoryPoints(trajectory.positions);
                const std::vector<double> flatVelocities =
                    flattenJointTrajectoryPoints(trajectory.velocities);

                std::cout << "Streaming "
                          << trajectory.positions.size()
                          << " validated MultiAxis::MovePVT waypoint(s), then waiting once for phase completion.\n";
                std::cout << "  Controller sample period used for time rounding: "
                          << std::fixed << std::setprecision(6)
                          << samplePeriodSeconds
                          << " sec.\n";
                std::cout << "  PVT emptyCount: "
                          << TrajectoryPvtEmptyCount
                          << " frame(s).\n";
                std::cout << "  Total planned phase time: "
                          << trajectory.totalSeconds
                          << " sec; point dt min/max: "
                          << minTrajectoryPointSeconds(trajectory)
                          << " / "
                          << maxTrajectoryPointSeconds(trajectory)
                          << " sec.\n";
                std::cout << "  Max specified waypoint velocity: "
                          << maxAbsTrajectoryVelocity(trajectory)
                          << " user-units/sec = "
                          << toDegrees(maxAbsTrajectoryVelocity(trajectory))
                          << " deg/sec.\n";

                const std::string multiAxisContext =
                    std::string("before PVT trajectory Cartesian-vector ") + phaseLabel + " phase";
                configureMultiAxisMotionAttributes(multiAxisContext.c_str());
                multiAxis_->MotionAttributeMaskOffSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskAPPEND);
                multiAxis_->MotionAttributeMaskOffSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskNO_WAIT);

                for (int index = 0; index < AxisCount; ++index)
                {
                    const std::string axisContext =
                        std::string("before PVT trajectory Cartesian-vector ") + phaseLabel + " phase";
                    configureAxisMotionAttributes(index, axisContext.c_str());
                }

                if (!CompactSegmentedExecutionEnabled)
                {
                    printAllAxisMotionStatus("All axes before PVT trajectory Cartesian-vector phase");
                }

                const uint16_t motionIdBefore = multiAxis_->MotionIdGet();

                multiAxis_->MovePVT(
                    flatPositions.data(),
                    flatVelocities.data(),
                    trajectory.times.data(),
                    static_cast<int32_t>(trajectory.positions.size()),
                    TrajectoryPvtEmptyCount,
                    false,
                    true);

                std::cout << "  MotionId "
                          << motionIdBefore
                          << " -> "
                          << multiAxis_->MotionIdGet()
                          << " (PVT stream).\n";

                const std::string startLabel =
                    std::string("PVT trajectory Cartesian-vector ") + phaseLabel + " MultiAxis::MovePVT";
                waitForAllAxisMotionStart(startLabel.c_str(), startingCommandPositions);

                const int phaseTimeoutMilliseconds = trajectoryBlockMotionTimeoutMs(trajectory);
                std::cout << "Waiting once for PVT "
                          << phaseLabel
                          << " phase completion. Timeout="
                          << phaseTimeoutMilliseconds
                          << " ms.\n";
                waitForMotionDone(phaseTimeoutMilliseconds);

                if (CompactSegmentedExecutionEnabled)
                {
                    const std::string actualLabel =
                        std::string("Actual positions after PVT trajectory Cartesian-vector ") + phaseLabel + " phase";
                    printActualPositions(actualLabel.c_str());
                }
                else
                {
                    const std::string statusLabel =
                        std::string("All axes after PVT trajectory Cartesian-vector ") + phaseLabel + " MotionDoneWait";
                    printAllAxisMotionStatus(statusLabel.c_str());
                }

                const std::string resetContext =
                    std::string("after PVT trajectory Cartesian-vector ") + phaseLabel + " phase";
                configureMultiAxisMotionAttributes(resetContext.c_str());
            };

            commandTrajectoryPhase("outbound", outboundSequence);
            commandTrajectoryPhase("return", returnSequence);
        }
        else if (AppendSegmentedExecutionEnabled)
        {
            const std::vector<JointVector> outboundSequence =
                makeOutboundSequenceFromSegmentPlan(segmentPlan);
            const std::vector<JointVector> returnSequence =
                makeReturnSequenceFromSegmentPlan(segmentPlan);

            if (outboundSequence.empty() || returnSequence.empty())
            {
                throw std::runtime_error("Queued segmented Cartesian sequence is empty.");
            }

            auto readAllAxisCommandPositions = [&]() {
                std::array<double, AxisCount> commandPositions{};

                for (int index = 0; index < AxisCount; ++index)
                {
                    commandPositions[index] = axes_[index]->CommandPositionGet();
                }

                return commandPositions;
            };

            auto commandQueuedPhase = [&](const char* phaseLabel, const std::vector<JointVector>& phaseSequence) {
                std::cout << "\n=== Queued segmented Cartesian-vector "
                          << phaseLabel
                          << " phase ===\n";
                std::cout << "Queueing "
                          << phaseSequence.size()
                          << " validated MultiAxis::MoveRelative command(s), then waiting once for phase completion.\n";

                const std::string multiAxisContext =
                    std::string("before queued segmented Cartesian-vector ") + phaseLabel + " phase";
                configureMultiAxisMotionAttributes(multiAxisContext.c_str());
                multiAxis_->MotionAttributeMaskOffSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskAPPEND);
                multiAxis_->MotionAttributeMaskOffSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskNO_WAIT);

                for (int index = 0; index < AxisCount; ++index)
                {
                    const std::string axisContext =
                        std::string("before queued segmented Cartesian-vector ") + phaseLabel + " phase";
                    configureAxisMotionAttributes(index, axisContext.c_str());
                }

                if (!CompactSegmentedExecutionEnabled)
                {
                    printAllAxisMotionStatus("All axes before queued segmented Cartesian-vector phase");
                }

                const std::array<double, AxisCount> startingCommandPositions =
                    readAllAxisCommandPositions();

                for (size_t stepIndex = 0; stepIndex < phaseSequence.size(); ++stepIndex)
                {
                    const JointVector& relativePosition = phaseSequence[stepIndex];

                    if (stepIndex == 1)
                    {
                        std::cout << "Enabling MultiAxis APPEND for remaining queued "
                                  << phaseLabel
                                  << " segment commands.\n";
                        multiAxis_->MotionAttributeMaskOnSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskAPPEND);
                        multiAxis_->MotionAttributeMaskOffSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskDELAY);
                        multiAxis_->MotionAttributeMaskOffSet(RR::RSIMotionAttrMask::RSIMotionAttrMaskNO_WAIT);
                        printMotionAttributeMasks("MultiAxis 6 queued append", multiAxis_);
                    }

                    JointVector degrees{};
                    double maxAbsDegrees = 0.0;
                    for (size_t axis = 0; axis < relativePosition.size(); ++axis)
                    {
                        degrees[axis] = toDegrees(relativePosition[axis]);
                        maxAbsDegrees = std::max(maxAbsDegrees, std::fabs(degrees[axis]));
                    }

                    std::cout << "Queued "
                              << phaseLabel
                              << " segment "
                              << (stepIndex + 1)
                              << " / "
                              << phaseSequence.size()
                              << ": max joint delta = "
                              << std::fixed << std::setprecision(6)
                              << maxAbsDegrees
                              << " deg, APPEND="
                              << (stepIndex == 0 ? "off" : "on")
                              << ", NO_WAIT=off.\n";

                    if (!CompactSegmentedExecutionEnabled)
                    {
                        std::cout << "Relative position array [J1..J6] in user units: ";
                        printJointVector(relativePosition);

                        std::cout << "Relative position array [J1..J6] in approx degrees: ";
                        printJointVector(degrees);
                    }

                    const uint16_t motionIdBefore = multiAxis_->MotionIdGet();

                    multiAxis_->MoveRelative(
                        relativePosition.data(),
                        velocity.data(),
                        acceleration.data(),
                        deceleration.data(),
                        jerk.data());

                    std::cout << "  MotionId "
                              << motionIdBefore
                              << " -> "
                              << multiAxis_->MotionIdGet()
                              << (stepIndex == 0 ? " (new phase motion)" : " (appended)")
                              << ".\n";
                }

                const std::string startLabel =
                    std::string("Queued segmented Cartesian-vector ") + phaseLabel + " MultiAxis::MoveRelative";
                waitForAllAxisMotionStart(startLabel.c_str(), startingCommandPositions);

                const int phaseTimeoutMilliseconds = queuedSequenceMotionTimeoutMs(phaseSequence);
                std::cout << "Waiting once for queued "
                          << phaseLabel
                          << " phase completion. Timeout="
                          << phaseTimeoutMilliseconds
                          << " ms.\n";
                waitForMotionDone(phaseTimeoutMilliseconds);

                if (CompactSegmentedExecutionEnabled)
                {
                    const std::string actualLabel =
                        std::string("Actual positions after queued segmented Cartesian-vector ") + phaseLabel + " phase";
                    printActualPositions(actualLabel.c_str());
                }
                else
                {
                    const std::string statusLabel =
                        std::string("All axes after queued segmented Cartesian-vector ") + phaseLabel + " MotionDoneWait";
                    printAllAxisMotionStatus(statusLabel.c_str());
                }

                const std::string resetContext =
                    std::string("after queued segmented Cartesian-vector ") + phaseLabel + " phase";
                configureMultiAxisMotionAttributes(resetContext.c_str());
            };

            commandQueuedPhase("outbound", outboundSequence);
            commandQueuedPhase("return", returnSequence);
        }
        else
        {
            for (size_t stepIndex = 0; stepIndex < relativeSequence.size(); ++stepIndex)
            {
            const JointVector& relativePosition = relativeSequence[stepIndex];

            std::cout << "\n=== Segmented Cartesian-vector MoveRelative step "
                      << (stepIndex + 1)
                      << " / "
                      << relativeSequence.size()
                      << " ===\n";

            if (stepIndex < static_cast<size_t>(segmentPlan.segmentCount))
            {
                std::cout << "Direction: outbound Cartesian segment "
                          << (stepIndex + 1)
                          << " / "
                          << segmentPlan.segmentCount
                          << "\n";
            }
            else
            {
                const size_t returnIndex = stepIndex - static_cast<size_t>(segmentPlan.segmentCount) + 1;
                std::cout << "Direction: return segment "
                          << returnIndex
                          << " / "
                          << segmentPlan.segmentCount
                          << "\n";
            }

            JointVector degrees{};
            double maxAbsDegrees = 0.0;
            for (size_t axis = 0; axis < relativePosition.size(); ++axis)
            {
                degrees[axis] = toDegrees(relativePosition[axis]);
                maxAbsDegrees = std::max(maxAbsDegrees, std::fabs(degrees[axis]));
            }

            if (CompactSegmentedExecutionEnabled)
            {
                std::cout << "Compact step summary: max joint delta = "
                          << std::fixed << std::setprecision(6)
                          << maxAbsDegrees
                          << " deg, velocity = "
                          << MotionVelocity
                          << " user-units/sec.\n";
            }
            else
            {
                std::cout << "Relative position array [J1..J6] in user units: ";
                printJointVector(relativePosition);

                std::cout << "Relative position array [J1..J6] in approx degrees: ";
                printJointVector(degrees);

                std::cout << "Velocity array [J1..J6] in user-units/sec: ";
                printJointVector(velocity);

                clearErrorLog("MotionController", controller_);
                clearErrorLog("MultiAxis 6", multiAxis_);

                for (int index = 0; index < AxisCount; ++index)
                {
                    clearErrorLog(("Axis " + std::to_string(index + 1)).c_str(), axes_[index]);
                }
            }

            configureMultiAxisMotionAttributes("before segmented Cartesian-vector MultiAxis::MoveRelative step");

            for (int index = 0; index < AxisCount; ++index)
            {
                configureAxisMotionAttributes(index, "before segmented Cartesian-vector MultiAxis::MoveRelative step");
            }

            if (!CompactSegmentedExecutionEnabled)
            {
                printAllAxisMotionStatus("All axes before segmented Cartesian-vector MoveRelative");
            }

            const uint16_t commandedMotionId = multiAxis_->MotionIdGet();
            std::array<double, AxisCount> startingCommandPositions{};

            for (int index = 0; index < AxisCount; ++index)
            {
                startingCommandPositions[index] = axes_[index]->CommandPositionGet();
            }

            if (!CompactSegmentedExecutionEnabled)
            {
                std::cout << "Commanding MultiAxis::MoveRelative for segmented Cartesian-vector joint step.\n";
                std::cout << "  MultiAxis commanded MotionId before call: " << commandedMotionId << "\n";
            }

            multiAxis_->MoveRelative(
                relativePosition.data(),
                velocity.data(),
                acceleration.data(),
                deceleration.data(),
                jerk.data());

            if (CompactSegmentedExecutionEnabled)
            {
                std::cout << "  MotionId " << commandedMotionId
                          << " -> " << multiAxis_->MotionIdGet()
                          << ". Waiting for done...\n";
            }
            else
            {
                std::cout << "  MultiAxis next MotionId after call: " << multiAxis_->MotionIdGet() << "\n";
                printAllAxisMotionStatus("All axes immediately after segmented Cartesian-vector MoveRelative");
            }

            waitForAllAxisMotionStart(
                "Segmented Cartesian-vector MultiAxis::MoveRelative",
                startingCommandPositions);

            if (!CompactSegmentedExecutionEnabled)
            {
                for (int sample = 0; sample < 8; ++sample)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(MotionStatusSampleMs));
                    printAllAxisProgressLine("Segmented Cartesian-vector live sample", sample + 1);
                }
            }

            waitForMotionDone(MotionTimeoutMs);

            if (CompactSegmentedExecutionEnabled)
            {
                std::cout << "  Step " << (stepIndex + 1)
                          << " complete.\n";
            }
            else
            {
                printActualPositions("Actual positions after segmented Cartesian-vector MoveRelative step");
                printAllAxisMotionStatus("All axes after segmented Cartesian-vector MotionDoneWait");
            }
            }
        }
    }
    catch (...)
    {
        if (errorLimitsTemporarilyChanged)
        {
            for (int index = 0; index < AxisCount; ++index)
            {
                axes_[index]->ErrorLimitActionSet(originalErrorLimitActions[index]);
                std::cout << "Restored Axis " << (index + 1)
                          << " ErrorLimitAction to "
                          << actionName(originalErrorLimitActions[index])
                          << " after exception.\n";
            }
        }

        throw;
    }

    if (errorLimitsTemporarilyChanged)
    {
        for (int index = 0; index < AxisCount; ++index)
        {
            axes_[index]->ErrorLimitActionSet(originalErrorLimitActions[index]);
            std::cout << "Restored Axis " << (index + 1)
                      << " ErrorLimitAction to "
                      << actionName(originalErrorLimitActions[index])
                      << ".\n";
        }
    }

    std::cout << "Segmented Cartesian-vector MultiAxis execution complete. Net commanded offsets are zero.\n";
    if (CompactSegmentedExecutionEnabled)
    {
        printActualPositions("Actual positions after compact segmented Cartesian-vector sequence");
    }

    printReturnToZeroReport("Return-to-zero check after segmented Cartesian-vector motion", true);

    printActualPositions("Actual positions before Cartesian-vector shutdown");
    printDiagnosticSnapshot("Before Cartesian-vector shutdown");

    disableAmplifiers();

    std::cout << "Guarded segmented Cartesian-vector motion complete.\n";
}


void Racer3BasicMotion::printMotionPlan() const
{
    if (CartesianVectorMotionEnabled)
    {
        std::cout << "\nGuarded Cartesian-vector IK motion plan\n";
        std::cout << "Purpose: compute custom OpenRAVE IK candidate(s) for a requested Cartesian delta, splitting larger requests into guarded absolute waypoints when needed.\n";
        std::cout << "Default behavior without --confirm-motion: compute and print the accepted/rejected segmented plan only; no amp enable and no motion.\n";
        std::cout << "With --confirm-motion: every segment must pass residual and command joint-delta gates before any motion is allowed.\n";
        std::cout << "Motion execution path: Cartesian waypoint segment(s) -> joint-vector user units -> selected execution mode -> reverse/return path -> return-to-zero check.\n";
        std::cout << "Validation gates per segment: residual norm <= "
                  << CandidateResidualNormAccept
                  << ", max residual component <= "
                  << CandidateMaxResidualComponentAccept
                  << ", and max command joint delta <= "
                  << CandidateMaxJointDeltaDegreesAccept
                  << " degrees.\n";
        std::cout << "Max adaptive segments: " << MaxCartesianSegments << ".\n";
        std::cout << "IK residual mode: " << ikResidualModeName() << ".\n";
        std::cout << "Segmented execution logging mode: " << compactSegmentedExecutionName() << ".\n";
        std::cout << "Segmented motion execution mode: " << segmentedMotionExecutionModeName() << ".\n";
        if (CompactSegmentedExecutionEnabled)
        {
            std::cout << "CompactMotion: skips per-segment live samples/status dumps during confirmed motion.\n";
        }
        if (AppendSegmentedExecutionEnabled)
        {
            std::cout << "AppendMotion: experimental queued APPEND execution, one MotionDoneWait per outbound/return phase; live execution still requires --confirm-motion.\n";
        }
        if (TrajectorySegmentedExecutionEnabled)
        {
            std::cout << "TrajectoryMotion: experimental MultiAxis::MovePVT streaming of validated joint waypoints, one MotionDoneWait per outbound/return phase; live execution still requires --confirm-motion.\n";
        }
        if (EndpointOnlyMotionEnabled)
        {
            std::cout << "EndpointOnly: skips segmented Cartesian waypoints and solves one final XYZ target, then uses smooth joint-space PVT.\n";
            std::cout << "EndpointOnly reaches the point but does not guarantee a straight Cartesian TCP path.\n";
        }
        if (SegmentGoalMotionEnabled)
        {
            std::cout << "SegmentGoal: segmented IK finds the final branch/goal, then one smooth joint-space PVT move goes to that final joint target.\n";
            std::cout << "SegmentGoal reaches the point but does not guarantee a straight Cartesian TCP path.\n";
        }
        if (PositionOnlyIkEnabled)
        {
            std::cout << "Position-only IK: roll/pitch/yaw are ignored by the solve and validation gates.\n";
        }
        printCartesianVector("Requested Cartesian delta", RequestedCartesianVector);
        std::cout << "\n";
        return;
    }

    if (KinematicsDryRunEnabled)
    {
        std::cout << "\nCustom Racer3 kinematics dry-run plan\n";
        std::cout << "Purpose: begin the custom articulated kinematics layer using the old rapidrobot OpenRAVE Racer3 model.\n";
        std::cout << "This mode connects to RMP and reads the current joints, but it does not enable amps or command motion.\n";
        std::cout << "Geometry source: racer3.kinbody.xml joint anchors/axes plus racer3.robot.xml tool point translation 0 0 1.012.\n";
        std::cout << "Current implementation: FK scaffold plus wide-J1 multi-seed numerical IK dry-run with residual/joint-delta gates. Motion execution is intentionally not enabled yet.\n";
        printCartesianVector("Requested Cartesian delta", RequestedCartesianVector);
        std::cout << "\n";
        return;
    }

    if (RobotPoseProbeEnabled)
    {
        std::cout << "\nRobot pose/FK API probe plan\n";
        std::cout << "Purpose: test read-only Cartesian Robot pose, joint, FK, and IK APIs.\n";
        std::cout << "This mode connects to RMP, loads/maps MultiAxis 6, creates configured Robot objects, clears faults, but does not enable amps or command motion.\n";
        std::cout << "Robots tested:\n";
        std::cout << "  1. LinearModelBuilder(\"RSI_Racer3\") with matching Axis labels + X/Y/Z/Roll/Pitch/Yaw mappings\n";
        std::cout << "  2. LinearModelBuilder(\"RSI_XYZABC_Millimeters\") with the same mappings\n";
        std::cout << "Read-only calls tested:\n";
        std::cout << "  JointsActualPositionsGet, JointsCommandPositionsGet, ActualPoseGet, CommandPoseGet, ActualPositionGet, CommandPositionGet, ForwardKinematics, InverseKinematics\n";
        std::cout << "This numeric version prints actual values and synthetic FK outputs so RSI_Racer3 can be compared against RSI_XYZABC_Millimeters.\n\n";
        return;
    }

    if (RobotModelProbeEnabled)
    {
        std::cout << "\nRobot model probe plan\n";
        std::cout << "Purpose: test RSI Cartesian Robot creation for the built-in Racer3 model.\n";
        std::cout << "This mode connects to RMP and loads/maps MultiAxis 6, but does not clear faults, enable amps, or command motion.\n";
        std::cout << "Probe candidates:\n";
        std::cout << "  1. LinearModelBuilder(\"RSI_Racer3\") with no JointAdd calls, expected to show the unconfigured-model error.\n";
        std::cout << "  2. LinearModelBuilder(\"RSI_Racer3\") with matching Axis UserLabel + X/Y/Z/Roll/Pitch/Yaw JointAdd mappings.\n";
        std::cout << "  3. LinearModelBuilder(\"RSI_XYZABC_Millimeters\") with matching Axis UserLabel + the same mappings as a known linear baseline.\n";
        std::cout << "  4. RobotCreate(controller, multiAxis, \"RSI_Racer3\") direct identifier probe.\n";
        std::cout << "A successful configured-builder probe means the Robot layer can be created, but it may still be a linear XYZABC model rather than true articulated Racer3 FK/IK.\n\n";
        return;
    }

    if (JointVectorMotionEnabled)
    {
        std::cout << "\nCustom joint-vector MultiAxis MoveRelative diagnostic motion plan\n";
        std::cout << "Startup path: scripts/start-racer3-rmp-and-run.ps1 runs rsiconfig first.\n";
        std::cout << "Enable path: LoadExistingMultiAxis(6), then AxisRemoveAll/AxisAdd, then enable all six drives.\n";
        std::cout << "Joint-vector path: after all six drives are enabled, MultiAxis 6 is remapped to Axis 1 through Axis 6.\n";
        std::cout << "Motion path: MultiAxis::MoveRelative arrays [J1..J6] using the requested custom relative vector.\n";
        std::cout << "All six axes scaling: 1.0 user unit = 1 physical revolution on each axis.\n";
        std::cout << "All six axes HomeAction are set to NONE in code.\n";
        std::cout << "All six axes ErrorLimitAction are temporarily set to NONE only during joint-vector motion.\n";
        std::cout << "The requested vector is commanded, then the negative vector is commanded to return to software zero.\n";
    }
    else if (AllMotionEnabled)
    {
        std::cout << "\nAll-axis synchronized MultiAxis MoveRelative diagnostic motion plan\n";
        std::cout << "Startup path: scripts/start-racer3-rmp-and-run.ps1 runs rsiconfig first.\n";
        std::cout << "Enable path: LoadExistingMultiAxis(6), then AxisRemoveAll/AxisAdd, then enable all six drives.\n";
        std::cout << "All-axis path: after all six drives are enabled, MultiAxis 6 is remapped to Axis 1 through Axis 6.\n";
        std::cout << "Motion path: MultiAxis::MoveRelative arrays [J1..J6] with the same relative range and velocity.\n";
        std::cout << "All six axes scaling: 1.0 user unit = 1 physical revolution on each axis.\n";
        std::cout << "All six axes HomeAction are set to NONE in code.\n";
        std::cout << "All six axes ErrorLimitAction are temporarily set to NONE only during all-motion.\n";
        std::cout << "All six axes receive synchronized motion commands. Cleanup disables each axis individually.\n";
    }
    else if (DualMotionEnabled)
    {
        std::cout << "\nAxis 5 + Axis 6 synchronized MultiAxis MoveRelative diagnostic motion plan\n";
        std::cout << "Startup path: scripts/start-racer3-rmp-and-run.ps1 runs rsiconfig first.\n";
        std::cout << "Enable path: LoadExistingMultiAxis(6), then AxisRemoveAll/AxisAdd, then enable all six drives.\n";
        std::cout << "Dual-axis isolation path: after all six drives are enabled, MultiAxis 6 is remapped to only Axis 5 and Axis 6.\n";
        std::cout << "Motion path: MultiAxis::MoveRelative arrays [J5, J6] with the same relative range and velocity.\n";
        std::cout << "Axis 5 and Axis 6 scaling: 1.0 user unit = 1 physical revolution on each axis.\n";
        std::cout << "Axis 5 and Axis 6 HomeAction are set to NONE in code.\n";
        std::cout << "Axis 5 and Axis 6 ErrorLimitAction are temporarily set to NONE only during dual-motion.\n";
        std::cout << "Only J5 and J6 receive motion commands. Cleanup disables each axis individually.\n";
    }
    else
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
    }
    std::cout << "Diagnostics: " << (DiagnosticsEnabled ? "FULL (--diagnostics enabled)" : "COMPACT (use --diagnostics for full dumps)") << "\n";
    if (JointVectorMotionEnabled)
    {
        std::cout << "Requested joint vector [J1..J6] in user units: ";
        printJointVector(RequestedJointVector);
        JointVector requestedDegrees{};
        for (size_t axis = 0; axis < RequestedJointVector.size(); ++axis)
        {
            requestedDegrees[axis] = toDegrees(RequestedJointVector[axis]);
        }
        std::cout << "Requested joint vector [J1..J6] in approx degrees: ";
        printJointVector(requestedDegrees);
        std::cout << "Max absolute joint delta = "
                  << maxAbsJointValue(RequestedJointVector)
                  << " user units = "
                  << toDegrees(maxAbsJointValue(RequestedJointVector))
                  << " degrees.\n";
    }
    else
    {
        std::cout << "Test step = "
                  << Axis6TestStepUserUnits
                  << " user units = "
                  << toDegrees(Axis6TestStepUserUnits)
                  << " degrees.\n";
    }

    std::array<JointVector, 2> plannedMoves{};

    if (JointVectorMotionEnabled)
    {
        plannedMoves = {
            RequestedJointVector,
            negateJointVector(RequestedJointVector)
        };
    }
    else if (AllMotionEnabled)
    {
        plannedMoves = {
            makeAllAxesVector(Axis6TestStepUserUnits),
            makeAllAxesVector(-Axis6TestStepUserUnits)
        };
    }
    else if (DualMotionEnabled)
    {
        plannedMoves = {
            makeAxis5And6Vector(Axis6TestStepUserUnits, Axis6TestStepUserUnits),
            makeAxis5And6Vector(-Axis6TestStepUserUnits, -Axis6TestStepUserUnits)
        };
    }
    else
    {
        plannedMoves = {
            makeAxis6OnlyVector(Axis6TestStepUserUnits),
            makeAxis6OnlyVector(-Axis6TestStepUserUnits)
        };
    }

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
    if (!DiagnosticsEnabled)
    {
        std::cout << "\n--- " << label << " (compact; use --diagnostics for full dump) ---\n";
        if (AllMotionEnabled)
        {
            printAllAxisProgressLine(label, 0);
        }
        else if (DualMotionEnabled && axes_[Axis5Index] && axes_[Axis6Index])
        {
            printDualAxisProgressLine(label, 0);
        }
        else
        {
            printMotionProgressLine(label, 0);
        }
        std::cout << "--- end compact snapshot ---\n";
        return;
    }

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


void Racer3BasicMotion::printAxis5And6MotionStatus(const char* label)
{
    if (!axes_[Axis5Index] || !axes_[Axis6Index])
    {
        std::cout << label << ": Axis 5 or Axis 6 is not initialized.\n";
        return;
    }

    try
    {
        std::cout << label
                  << " | J5 CmdPos=" << std::fixed << std::setprecision(6) << axes_[Axis5Index]->CommandPositionGet()
                  << " ActPos=" << axes_[Axis5Index]->ActualPositionGet()
                  << " CmdVel=" << axes_[Axis5Index]->CommandVelocityGet()
                  << " ActVel=" << axes_[Axis5Index]->ActualVelocityGet()
                  << " | J6 CmdPos=" << axes_[Axis6Index]->CommandPositionGet()
                  << " ActPos=" << axes_[Axis6Index]->ActualPositionGet()
                  << " CmdVel=" << axes_[Axis6Index]->CommandVelocityGet()
                  << " ActVel=" << axes_[Axis6Index]->ActualVelocityGet()
                  << '\n';
    }
    catch (const RR::RsiError& error)
    {
        std::cout << label << " Axis 5/6 numeric status threw RapidCode error: " << error.text << "\n";
        std::cout << "Function: " << error.functionName << "\n";
    }
}


bool Racer3BasicMotion::printReturnToZeroReport(const char* label, bool throwOnFail)
{
    std::cout << "\n" << label << "\n";
    std::cout << "Software-zero target is 0.0 user units on every axis.\n";
    std::cout << "Warning tolerance: "
              << ReturnWarnToleranceUserUnits
              << " user units = "
              << toDegrees(ReturnWarnToleranceUserUnits)
              << " degrees.\n";
    std::cout << "Fail tolerance:    "
              << ReturnFailToleranceUserUnits
              << " user units = "
              << toDegrees(ReturnFailToleranceUserUnits)
              << " degrees.\n";

    bool anyWarning = false;
    bool anyFail = false;

    std::cout << std::fixed << std::setprecision(6);

    for (int index = 0; index < AxisCount; ++index)
    {
        RR::Axis* axis = axes_[index];

        if (!axis)
        {
            std::cout << "  J" << (index + 1) << ": <null axis> FAIL\n";
            anyFail = true;
            continue;
        }

        try
        {
            const double commandPosition = axis->CommandPositionGet();
            const double actualPosition = axis->ActualPositionGet();
            const double commandError = commandPosition;
            const double actualError = actualPosition;
            const double absoluteActualError = std::fabs(actualError);

            const char* status = "OK";
            if (absoluteActualError > ReturnFailToleranceUserUnits)
            {
                status = "FAIL";
                anyFail = true;
            }
            else if (absoluteActualError > ReturnWarnToleranceUserUnits)
            {
                status = "WARN";
                anyWarning = true;
            }

            std::cout << "  J" << (index + 1)
                      << " CmdPos=" << commandPosition
                      << " ActPos=" << actualPosition
                      << " CmdErr=" << commandError
                      << " ActErr=" << actualError
                      << " ActErrDeg=" << toDegrees(actualError)
                      << " => " << status
                      << "\n";
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  J" << (index + 1)
                      << ": return-to-zero read threw RapidCode error: "
                      << error.text
                      << " ("
                      << error.functionName
                      << ") FAIL\n";
            anyFail = true;
        }
        catch (...)
        {
            std::cout << "  J" << (index + 1)
                      << ": return-to-zero read threw unknown exception. FAIL\n";
            anyFail = true;
        }
    }

    if (anyFail)
    {
        std::cout << "Return-to-zero result: FAIL. One or more axes are outside the fail tolerance.\n";

        if (throwOnFail)
        {
            throw std::runtime_error("Return-to-zero check failed. Axis actual position is outside --return-fail tolerance.");
        }

        return false;
    }

    if (anyWarning)
    {
        std::cout << "Return-to-zero result: WARN. Motion completed, but one or more axes are outside the warning tolerance.\n";
    }
    else
    {
        std::cout << "Return-to-zero result: OK. All axes are within the warning tolerance.\n";
    }

    return true;
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


void Racer3BasicMotion::printDualAxisProgressLine(const char* label, int sampleNumber)
{
    if (!multiAxis_ || !axes_[Axis5Index] || !axes_[Axis6Index])
    {
        std::cout << label << " sample " << sampleNumber << ": motion objects are not initialized.\n";
        return;
    }

    try
    {
        const RR::RSIState multiState = multiAxis_->StateGet();
        const RR::RSIState axis5State = axes_[Axis5Index]->StateGet();
        const RR::RSIState axis6State = axes_[Axis6Index]->StateGet();

        std::cout << label
                  << " sample " << sampleNumber
                  << " | MultiAxis(State=" << stateName(multiState)
                  << ", MotionId=" << multiAxis_->MotionIdGet()
                  << ", Exec=" << multiAxis_->MotionIdExecutingGet()
                  << ", Done=" << boolText(multiAxis_->MotionDoneGet())
                  << ") J5(State=" << stateName(axis5State)
                  << ", CmdPos=" << std::fixed << std::setprecision(6) << axes_[Axis5Index]->CommandPositionGet()
                  << ", CmdVel=" << axes_[Axis5Index]->CommandVelocityGet()
                  << ", ActPos=" << axes_[Axis5Index]->ActualPositionGet()
                  << ", ActVel=" << axes_[Axis5Index]->ActualVelocityGet()
                  << ") J6(State=" << stateName(axis6State)
                  << ", CmdPos=" << axes_[Axis6Index]->CommandPositionGet()
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


void Racer3BasicMotion::printAllAxisMotionStatus(const char* label)
{
    std::cout << label << "\n";

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            std::cout << "  J" << (index + 1) << ": <null>\n";
            continue;
        }

        try
        {
            std::cout << "  J" << (index + 1)
                      << " CmdPos=" << std::fixed << std::setprecision(6) << axes_[index]->CommandPositionGet()
                      << " ActPos=" << axes_[index]->ActualPositionGet()
                      << " CmdVel=" << axes_[index]->CommandVelocityGet()
                      << " ActVel=" << axes_[index]->ActualVelocityGet()
                      << " State=" << stateName(axes_[index]->StateGet())
                      << " Done=" << boolText(axes_[index]->MotionDoneGet())
                      << "\n";
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  J" << (index + 1)
                      << " status threw RapidCode error: " << error.text
                      << " (" << error.functionName << ")\n";
        }
    }
}

void Racer3BasicMotion::printAllAxisProgressLine(const char* label, int sampleNumber)
{
    if (!multiAxis_)
    {
        std::cout << label << " sample " << sampleNumber << ": MultiAxis object is not initialized.\n";
        return;
    }

    try
    {
        const RR::RSIState multiState = multiAxis_->StateGet();

        std::cout << label
                  << " sample " << sampleNumber
                  << " | MultiAxis(State=" << stateName(multiState)
                  << ", MotionId=" << multiAxis_->MotionIdGet()
                  << ", Exec=" << multiAxis_->MotionIdExecutingGet()
                  << ", Done=" << boolText(multiAxis_->MotionDoneGet())
                  << ")";

        for (int index = 0; index < AxisCount; ++index)
        {
            if (!axes_[index])
            {
                std::cout << " J" << (index + 1) << "(<null>)";
                continue;
            }

            std::cout << " J" << (index + 1)
                      << "(State=" << stateName(axes_[index]->StateGet())
                      << ", CmdPos=" << std::fixed << std::setprecision(6) << axes_[index]->CommandPositionGet()
                      << ", CmdVel=" << axes_[index]->CommandVelocityGet()
                      << ", ActPos=" << axes_[index]->ActualPositionGet()
                      << ", ActVel=" << axes_[index]->ActualVelocityGet()
                      << ")";
        }

        std::cout << "\n";
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


void Racer3BasicMotion::waitForDualAxisMotionStart(
    const char* label,
    double startingAxis5CommandPosition,
    double startingAxis6CommandPosition)
{
    if (!multiAxis_ || !axes_[Axis5Index] || !axes_[Axis6Index])
    {
        throw std::runtime_error("Dual-axis motion objects are not initialized.");
    }

    std::cout << "Watching for " << label << " to generate a synchronized Axis 5/6 trajectory...\n";

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(MotionStartTimeoutMs);
    int sampleNumber = 0;

    while (std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(MotionStartSampleMs));
        ++sampleNumber;

        printDualAxisProgressLine("Dual start-watch", sampleNumber);

        const RR::RSIState multiState = multiAxis_->StateGet();
        const RR::RSIState axis5State = axes_[Axis5Index]->StateGet();
        const RR::RSIState axis6State = axes_[Axis6Index]->StateGet();
        const double axis5CommandPosition = axes_[Axis5Index]->CommandPositionGet();
        const double axis6CommandPosition = axes_[Axis6Index]->CommandPositionGet();
        const double axis5CommandVelocity = axes_[Axis5Index]->CommandVelocityGet();
        const double axis6CommandVelocity = axes_[Axis6Index]->CommandVelocityGet();

        const bool stateMoving =
            multiState == RR::RSIState::RSIStateMOVING ||
            axis5State == RR::RSIState::RSIStateMOVING ||
            axis6State == RR::RSIState::RSIStateMOVING;
        const bool axis5CommandPositionChanged =
            std::fabs(axis5CommandPosition - startingAxis5CommandPosition) > 1e-7;
        const bool axis6CommandPositionChanged =
            std::fabs(axis6CommandPosition - startingAxis6CommandPosition) > 1e-7;
        const bool axis5CommandVelocityNonZero = std::fabs(axis5CommandVelocity) > 1e-9;
        const bool axis6CommandVelocityNonZero = std::fabs(axis6CommandVelocity) > 1e-9;

        if (stateMoving ||
            axis5CommandPositionChanged ||
            axis6CommandPositionChanged ||
            axis5CommandVelocityNonZero ||
            axis6CommandVelocityNonZero)
        {
            std::cout << label << " started: "
                      << "stateMoving=" << boolText(stateMoving)
                      << ", axis5CommandPositionChanged=" << boolText(axis5CommandPositionChanged)
                      << ", axis6CommandPositionChanged=" << boolText(axis6CommandPositionChanged)
                      << ", axis5CommandVelocityNonZero=" << boolText(axis5CommandVelocityNonZero)
                      << ", axis6CommandVelocityNonZero=" << boolText(axis6CommandVelocityNonZero)
                      << ".\n";
            return;
        }
    }

    printDiagnosticSnapshot("Dual-axis command accepted but no synchronized trajectory appeared");

    try
    {
        std::cout << "Aborting dual-axis accepted-but-not-started command before error exit...\n";
        multiAxis_->Abort();
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "Dual-axis abort after non-started command threw RapidCode error: "
                  << error.text
                  << " ("
                  << error.functionName
                  << ")\n";
    }

    throw std::runtime_error(
        "Dual-axis command was accepted, but Axis 5/6 command position/velocity and motion state did not change.");
}


void Racer3BasicMotion::waitForAllAxisMotionStart(
    const char* label,
    const std::array<double, AxisCount>& startingCommandPositions)
{
    if (!multiAxis_)
    {
        throw std::runtime_error("All-axis motion objects are not initialized.");
    }

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            throw std::runtime_error("Axis " + std::to_string(index + 1) + " is not initialized.");
        }
    }

    std::cout << "Watching for " << label << " to generate a synchronized all-axis trajectory...\n";

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(MotionStartTimeoutMs);
    int sampleNumber = 0;

    while (std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(MotionStartSampleMs));
        ++sampleNumber;

        printAllAxisProgressLine("All-axis start-watch", sampleNumber);

        const RR::RSIState multiState = multiAxis_->StateGet();
        bool stateMoving = multiState == RR::RSIState::RSIStateMOVING;
        bool anyCommandPositionChanged = false;
        bool anyCommandVelocityNonZero = false;

        for (int index = 0; index < AxisCount; ++index)
        {
            const RR::RSIState axisState = axes_[index]->StateGet();
            const double commandPosition = axes_[index]->CommandPositionGet();
            const double commandVelocity = axes_[index]->CommandVelocityGet();

            stateMoving = stateMoving || axisState == RR::RSIState::RSIStateMOVING;
            anyCommandPositionChanged = anyCommandPositionChanged ||
                std::fabs(commandPosition - startingCommandPositions[index]) > 1e-7;
            anyCommandVelocityNonZero = anyCommandVelocityNonZero ||
                std::fabs(commandVelocity) > 1e-9;
        }

        if (stateMoving || anyCommandPositionChanged || anyCommandVelocityNonZero)
        {
            std::cout << label << " started: "
                      << "stateMoving=" << boolText(stateMoving)
                      << ", anyCommandPositionChanged=" << boolText(anyCommandPositionChanged)
                      << ", anyCommandVelocityNonZero=" << boolText(anyCommandVelocityNonZero)
                      << ".\n";
            return;
        }
    }

    printDiagnosticSnapshot("All-axis command accepted but no synchronized trajectory appeared");

    try
    {
        std::cout << "Aborting all-axis accepted-but-not-started command before error exit...\n";
        multiAxis_->Abort();
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "All-axis abort after non-started command threw RapidCode error: "
                  << error.text
                  << " ("
                  << error.functionName
                  << ")\n";
    }

    throw std::runtime_error(
        "All-axis command was accepted, but command position/velocity and motion state did not change.");
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
