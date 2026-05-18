#include "Racer3BasicMotion.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <rsi.h>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

namespace RR = RSI::RapidCode;

namespace
{
constexpr double DefaultStepUserUnits = 0.05;
constexpr double DefaultVelocityUserUnitsPerSecond = 0.05;
constexpr double DefaultReturnWarnToleranceUserUnits = 0.00025;
constexpr double DefaultReturnFailToleranceUserUnits = 0.00100;
constexpr double MaxRecommendedStepUserUnits = 0.25;
constexpr int AxisCount = 6;
constexpr double MaxRecommendedJointVectorUserUnits = 0.25;

void printUsage()
{
    std::cout
        << "Racer3 RMP Basic Motion Demo\n\n"
        << "Usage:\n"
        << "  racer3-basic-motion --dry-run [--step 0.05] [--velocity 0.05] [--diagnostics]\n"
        << "  racer3-basic-motion --enable-only [--diagnostics]\n"
        << "  racer3-basic-motion --tiny-motion --confirm-motion [--step 0.05] [--velocity 0.05] [--diagnostics]\n"
        << "  racer3-basic-motion --dual-motion --confirm-motion [--step 0.05] [--velocity 0.05] [--diagnostics]\n"
        << "  racer3-basic-motion --all-motion --confirm-motion [--step 0.01] [--velocity 0.02] [--diagnostics]\n"
        << "  racer3-basic-motion --joint-vector --confirm-motion --joints j1,j2,j3,j4,j5,j6 [--velocity 0.02] [--return-warn 0.00025] [--return-fail 0.001] [--diagnostics]\n"
        << "  racer3-basic-motion --robot-model-probe [--diagnostics]\n"
        << "  racer3-basic-motion --robot-pose-probe [--diagnostics]\n"
        << "  racer3-basic-motion --kinematics-dry-run [--cartesian dx,dy,dz,dr,dp,dy] [--diagnostics]\n"
        << "  racer3-basic-motion --cartesian-vector [--position-only] [--compact-motion] [--trajectory-motion] [--endpoint-only] [--segment-goal] [--confirm-motion] --cartesian dx,dy,dz,dr,dp,dy [--velocity 0.02] [--diagnostics]\n\n"
        << "  racer3-basic-motion --cartesian-trace --position-only --endpoint-only [--compact-motion] [--confirm-motion] --cartesian-waypoints \"dx,dy,dz,dr,dp,dy;...\" [--velocity 0.02] [--diagnostics]\n"
        << "  racer3-basic-motion --session-server\n\n"
        << "Modes:\n"
        << "  --dry-run          Print the planned sequence only. No RMP connection.\n"
        << "  --enable-only      Connect, clear faults, enable, wait, disable. This is the default.\n"
        << "  --tiny-motion      Enable all drives, isolate J6, and run a tiny J6-only move.\n"
        << "  --dual-motion      Enable all drives, remap MultiAxis to J5+J6, and move both together.\n"
        << "  --all-motion       Enable all drives, remap MultiAxis to J1-J6, and move all together.\n"
        << "  --joint-vector     Enable all drives, remap MultiAxis to J1-J6, and move a custom joint vector.\n"
        << "  --robot-model-probe Connect/load MultiAxis and probe the RSI_Racer3 Cartesian Robot model. No amp enable or motion.\n"
        << "  --robot-pose-probe  Connect/load MultiAxis and probe read-only Robot pose/FK/IK APIs. No amp enable or motion.\n"
        << "  --kinematics-dry-run Connect/read joints and run the custom OpenRAVE Racer3 FK scaffold. No amp enable or motion.\n"
        << "  --cartesian-vector Compute a guarded Cartesian IK candidate; with --confirm-motion, execute only if validation gates pass.\n"
        << "  --cartesian-trace Validate a multi-waypoint Cartesian trace; with --confirm-motion, stream the validated joint waypoints as one outbound PVT phase, then return to software zero.\n"
        << "  --session-server Start a persistent local armed session. Connects RMP, enables amps once, accepts status/stop/shutdown, and keeps amps enabled until shutdown. Trace/jog motion commands are rejected in this phase.\n"
        << "  --position-only   For --cartesian-vector, solve and validate only XYZ position. Roll/pitch/yaw residual is printed but not gated.\n"
        << "  --compact-motion For --cartesian-vector confirmed segmented motion, skip per-segment live samples/status dumps.\n"
        << "  --append-motion  Experimental: queue segmented MoveRelative commands with APPEND.\n"
        << "  --trajectory-motion Experimental: stream validated segment endpoints with MultiAxis::MovePVT.\n"
        << "  --endpoint-only  Experimental: solve one final XYZ target and run smooth joint-space PVT. Not a straight Cartesian TCP path.\n"
        << "  --segment-goal  Experimental: use segmented IK only to find final goal, then run smooth joint-space PVT.\n"
        << "  --confirm-motion   Required safety acknowledgement for any real motion.\n"
        << "  --diagnostics      Print full diagnostic dumps. Default output is compact.\n"
        << "  --step <value>     Relative move in user units for tiny/dual/all modes. Default 0.05.\n"
        << "  --joints <list>    Six comma-separated user-unit deltas for --joint-vector, e.g. 0,0,0,0,0.005,0.005.\n"
        << "  --cartesian <list> Six comma-separated Cartesian deltas for --kinematics-dry-run or --cartesian-vector: dx,dy,dz meters and droll,dpitch,dyaw radians.\n"
        << "  --cartesian-waypoints <list> Semicolon-separated Cartesian waypoint deltas for --cartesian-trace. Each waypoint is dx,dy,dz,droll,dpitch,dyaw from the original software-zero start pose.\n"
        << "  --velocity <value> Velocity in user-units/sec. Default 0.05.\n"
        << "  --return-warn <value> Warn if final absolute actual position exceeds this user-unit tolerance. Default 0.00025.\n"
        << "  --return-fail <value> Fail if final absolute actual position exceeds this user-unit tolerance. Default 0.001.\n"
        << "  --help             Show this help text.\n\n"
        << "Notes:\n"
        << "  1.0 user unit = one physical revolution on each configured axis.\n"
        << "  0.05 user units = 18 degrees.\n";
}

bool hasArg(const std::vector<std::string>& args, const std::string& value)
{
    for (const auto& arg : args)
    {
        if (arg == value)
        {
            return true;
        }
    }
    return false;
}

bool startsWith(const std::string& value, const std::string& prefix)
{
    return value.rfind(prefix, 0) == 0;
}

double parseDoubleValue(const std::string& text, const std::string& optionName)
{
    try
    {
        size_t parsed = 0;
        const double value = std::stod(text, &parsed);
        if (parsed != text.size())
        {
            throw std::invalid_argument("trailing characters");
        }
        return value;
    }
    catch (const std::exception&)
    {
        throw std::runtime_error("Invalid value for " + optionName + ": " + text);
    }
}

double getDoubleOption(
    const std::vector<std::string>& args,
    const std::string& longName,
    double defaultValue)
{
    const std::string equalsPrefix = longName + "=";

    for (size_t index = 0; index < args.size(); ++index)
    {
        const std::string& arg = args[index];

        if (startsWith(arg, equalsPrefix))
        {
            return parseDoubleValue(arg.substr(equalsPrefix.size()), longName);
        }

        if (arg == longName)
        {
            if (index + 1 >= args.size())
            {
                throw std::runtime_error("Missing value after " + longName);
            }
            return parseDoubleValue(args[index + 1], longName);
        }
    }

    return defaultValue;
}

std::string getStringOption(
    const std::vector<std::string>& args,
    const std::string& longName,
    const std::string& defaultValue)
{
    const std::string equalsPrefix = longName + "=";

    for (size_t index = 0; index < args.size(); ++index)
    {
        const std::string& arg = args[index];

        if (startsWith(arg, equalsPrefix))
        {
            return arg.substr(equalsPrefix.size());
        }

        if (arg == longName)
        {
            if (index + 1 >= args.size())
            {
                throw std::runtime_error("Missing value after " + longName);
            }
            return args[index + 1];
        }
    }

    return defaultValue;
}

std::array<double, AxisCount> parseSixValueVector(const std::string& text, const std::string& optionName)
{
    std::array<double, AxisCount> values{};

    std::stringstream stream(text);
    std::string token;
    int index = 0;

    while (std::getline(stream, token, ','))
    {
        if (index >= AxisCount)
        {
            throw std::runtime_error(optionName + " must contain exactly six comma-separated values.");
        }

        values[index] = parseDoubleValue(token, optionName);
        ++index;
    }

    if (index != AxisCount)
    {
        throw std::runtime_error(optionName + " must contain exactly six comma-separated values.");
    }

    return values;
}

std::array<double, AxisCount> parseJointVector(const std::string& text)
{
    return parseSixValueVector(text, "--joints");
}

std::array<double, AxisCount> parseCartesianVector(const std::string& text)
{
    return parseSixValueVector(text, "--cartesian");
}

std::vector<std::array<double, AxisCount>> parseCartesianWaypoints(const std::string& text)
{
    std::vector<std::array<double, AxisCount>> waypoints;
    std::stringstream stream(text);
    std::string waypointText;

    while (std::getline(stream, waypointText, ';'))
    {
        if (waypointText.empty())
        {
            continue;
        }

        waypoints.push_back(parseSixValueVector(waypointText, "--cartesian-waypoints"));
    }

    if (waypoints.empty())
    {
        throw std::runtime_error("--cartesian-waypoints must contain at least one waypoint.");
    }

    return waypoints;
}

bool jointVectorHasMotion(const std::array<double, AxisCount>& values)
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

double maxAbsJointVector(const std::array<double, AxisCount>& values)
{
    double result = 0.0;

    for (double value : values)
    {
        result = std::max(result, std::fabs(value));
    }

    return result;
}



std::string normalizeSessionCommand(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character)
    {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

void writeSessionEvent(const std::string& json)
{
    std::cout << json << std::endl;
    std::cout.flush();
}

std::string escapeJsonText(const std::string& value)
{
    std::string result;
    result.reserve(value.size());

    for (char character : value)
    {
        switch (character)
        {
        case '\\':
            result += "\\\\";
            break;
        case '"':
            result += "\\\"";
            break;
        case '\n':
            result += "\\n";
            break;
        case '\r':
            result += "\\r";
            break;
        case '\t':
            result += "\\t";
            break;
        default:
            result += character;
            break;
        }
    }

    return result;
}

std::string jsonStringField(const std::string& text, const std::string& fieldName)
{
    const std::string key = "\"" + fieldName + "\"";
    const size_t keyPosition = text.find(key);
    if (keyPosition == std::string::npos)
    {
        return {};
    }

    const size_t colonPosition = text.find(':', keyPosition + key.size());
    if (colonPosition == std::string::npos)
    {
        return {};
    }

    size_t start = text.find('"', colonPosition + 1);
    if (start == std::string::npos)
    {
        return {};
    }
    ++start;

    std::string value;
    bool escaped = false;
    for (size_t index = start; index < text.size(); ++index)
    {
        const char character = text[index];
        if (escaped)
        {
            value += character;
            escaped = false;
            continue;
        }

        if (character == '\\')
        {
            escaped = true;
            continue;
        }

        if (character == '"')
        {
            return value;
        }

        value += character;
    }

    return {};
}

double jsonNumberField(const std::string& text, const std::string& fieldName, double defaultValue)
{
    const std::string key = "\"" + fieldName + "\"";
    const size_t keyPosition = text.find(key);
    if (keyPosition == std::string::npos)
    {
        return defaultValue;
    }

    const size_t colonPosition = text.find(':', keyPosition + key.size());
    if (colonPosition == std::string::npos)
    {
        return defaultValue;
    }

    size_t start = colonPosition + 1;
    while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start])))
    {
        ++start;
    }

    size_t end = start;
    while (end < text.size())
    {
        const char character = text[end];
        if (!(std::isdigit(static_cast<unsigned char>(character)) || character == '-' || character == '+' || character == '.' || character == 'e' || character == 'E'))
        {
            break;
        }
        ++end;
    }

    if (start == end)
    {
        return defaultValue;
    }

    return parseDoubleValue(text.substr(start, end - start), fieldName);
}

bool jsonBoolField(const std::string& text, const std::string& fieldName, bool defaultValue)
{
    const std::string key = "\"" + fieldName + "\"";
    const size_t keyPosition = text.find(key);
    if (keyPosition == std::string::npos)
    {
        return defaultValue;
    }

    const size_t colonPosition = text.find(':', keyPosition + key.size());
    if (colonPosition == std::string::npos)
    {
        return defaultValue;
    }

    size_t start = colonPosition + 1;
    while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start])))
    {
        ++start;
    }

    if (text.compare(start, 4, "true") == 0)
    {
        return true;
    }

    if (text.compare(start, 5, "false") == 0)
    {
        return false;
    }

    return defaultValue;
}

int runSessionServer()
{
    writeSessionEvent("{\"type\":\"session_starting\",\"state\":\"starting\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Session server starting. Connecting RMP and enabling amps once for persistent armed session.\"}");

    Racer3BasicMotion sessionMotion;

    try
    {
        sessionMotion.startArmedSession(DefaultVelocityUserUnitsPerSecond, false);
        writeSessionEvent("{\"type\":\"session_ready\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Persistent armed session ready. Amps are enabled and will stay enabled until Shutdown Session. Trace and jog commands are available in this phase.\"}");
    }
    catch (const RR::RsiError& error)
    {
        writeSessionEvent(std::string("{\"type\":\"session_error\",\"state\":\"faulted\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Failed to start armed session RapidCode error: ") + error.text + " (" + error.functionName + "). Shutdown attempted.\"}");
        sessionMotion.shutdownArmedSession();
        return 1;
    }
    catch (const std::exception& error)
    {
        writeSessionEvent(std::string("{\"type\":\"session_error\",\"state\":\"faulted\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Failed to start armed session: ") + error.what() + ". Shutdown attempted.\"}");
        sessionMotion.shutdownArmedSession();
        return 1;
    }
    catch (...)
    {
        writeSessionEvent("{\"type\":\"session_error\",\"state\":\"faulted\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Failed to start armed session with unknown exception. Shutdown attempted.\"}");
        sessionMotion.shutdownArmedSession();
        return 1;
    }

    std::string line;
    while (std::getline(std::cin, line))
    {
        const std::string command = normalizeSessionCommand(line);

        if (command.empty())
        {
            continue;
        }

        if (command.find("shutdown") != std::string::npos)
        {
            writeSessionEvent("{\"type\":\"session_shutdown_starting\",\"state\":\"shutting_down\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Shutdown requested. Disabling amps, clearing faults, and releasing controller.\"}");
            sessionMotion.shutdownArmedSession();
            writeSessionEvent("{\"type\":\"session_shutdown\",\"state\":\"exiting\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Persistent armed session shut down. Amps disabled and controller released.\"}");
            return 0;
        }

        if (command.find("status") != std::string::npos || command.find("hello") != std::string::npos)
        {
            writeSessionEvent("{\"type\":\"session_status\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Persistent armed session is alive. Amps are enabled. Trace and jog commands are available.\"}");
            continue;
        }

        if (command.find("stop") != std::string::npos)
        {
            try
            {
                sessionMotion.stopArmedSessionMotion();
                writeSessionEvent("{\"type\":\"session_stop_ack\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Stop acknowledged. Any active MultiAxis motion was aborted. Amps remain enabled for the armed session.\"}");
            }
            catch (const std::exception& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_error\",\"state\":\"faulted\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Stop failed: ") + error.what() + "\"}");
            }
            continue;
        }

        if (command.find("trace") != std::string::npos)
        {
            const std::string cartesianTrace = jsonStringField(line, "cartesianTrace");
            const double velocity = jsonNumberField(line, "velocity", DefaultVelocityUserUnitsPerSecond);
            const bool returnToZero = jsonBoolField(line, "returnToZero", true);

            try
            {
                if (cartesianTrace.empty())
                {
                    throw std::runtime_error("trace command is missing cartesianTrace.");
                }

                const std::vector<std::array<double, AxisCount>> waypoints = parseCartesianWaypoints(cartesianTrace);
                if (waypoints.empty())
                {
                    throw std::runtime_error("trace command did not contain any valid waypoints.");
                }

                writeSessionEvent("{\"type\":\"session_trace_starting\",\"state\":\"motion_running\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session trace command accepted. Validating and streaming trace while keeping amps enabled.\"}");
                sessionMotion.runArmedSessionTrace(waypoints, velocity, returnToZero);
                writeSessionEvent("{\"type\":\"session_trace_complete\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session trace complete. Amps remain enabled for additional session commands.\"}");
            }
            catch (const RR::RsiError& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_trace_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session trace RapidCode error: ") + escapeJsonText(error.text) + " (" + escapeJsonText(error.functionName) + "). Amps remain enabled; use Stop Motion or Shutdown Session if needed.\"}");
            }
            catch (const std::exception& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_trace_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session trace failed: ") + escapeJsonText(error.what()) + ". Amps remain enabled; use Stop Motion or Shutdown Session if needed.\"}");
            }

            continue;
        }

        if (command.find("cartesian_jog") != std::string::npos || command.find("jog") != std::string::npos)
        {
            const double dx = jsonNumberField(line, "dx", 0.0);
            const double dy = jsonNumberField(line, "dy", 0.0);
            const double dz = jsonNumberField(line, "dz", 0.0);
            const double velocity = jsonNumberField(line, "velocity", DefaultVelocityUserUnitsPerSecond);

            try
            {
                constexpr double MaxJogStepMeters = 0.025;
                if (velocity <= 0.0)
                {
                    throw std::runtime_error("cartesian_jog velocity must be greater than zero.");
                }

                if (std::abs(dx) > MaxJogStepMeters || std::abs(dy) > MaxJogStepMeters || std::abs(dz) > MaxJogStepMeters)
                {
                    throw std::runtime_error("cartesian_jog step exceeds the guarded maximum of 0.025 meters on one or more axes.");
                }

                if (dx == 0.0 && dy == 0.0 && dz == 0.0)
                {
                    throw std::runtime_error("cartesian_jog requires a non-zero dx, dy, or dz.");
                }

                const std::vector<std::array<double, AxisCount>> waypoint = {{{dx, dy, dz, 0.0, 0.0, 0.0}}};
                writeSessionEvent("{\"type\":\"session_jog_starting\",\"state\":\"motion_running\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session jog accepted. Running one validated Cartesian jog while keeping amps enabled.\"}");
                sessionMotion.runArmedSessionTrace(waypoint, velocity, false);
                writeSessionEvent("{\"type\":\"session_jog_complete\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session jog complete. Final pose is held and amps remain enabled.\"}");
            }
            catch (const RR::RsiError& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_jog_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session jog RapidCode error: ") + escapeJsonText(error.text) + " (" + escapeJsonText(error.functionName) + "). Amps remain enabled; use Stop Motion or Shutdown Session if needed.\"}");
            }
            catch (const std::exception& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_jog_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session jog failed: ") + escapeJsonText(error.what()) + ". Amps remain enabled; use Stop Motion or Shutdown Session if needed.\"}");
            }

            continue;
        }

        writeSessionEvent("{\"type\":\"session_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Unknown armed session command. Supported now: hello, status, trace, cartesian_jog, stop, shutdown.\"}");
    }

    writeSessionEvent("{\"type\":\"session_input_closed\",\"state\":\"shutting_down\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session input closed. Disabling amps and exiting.\"}");
    sessionMotion.shutdownArmedSession();
    return 0;
}

void validateOptions(const Racer3RunOptions& options)
{
    if (options.stepUserUnits <= 0.0)
    {
        throw std::runtime_error("--step must be greater than zero.");
    }

    if (options.velocityUserUnitsPerSecond <= 0.0)
    {
        throw std::runtime_error("--velocity must be greater than zero.");
    }

    if (options.returnWarnToleranceUserUnits < 0.0)
    {
        throw std::runtime_error("--return-warn must be zero or greater.");
    }

    if (options.returnFailToleranceUserUnits <= 0.0)
    {
        throw std::runtime_error("--return-fail must be greater than zero.");
    }

    if (options.returnWarnToleranceUserUnits > options.returnFailToleranceUserUnits)
    {
        throw std::runtime_error("--return-warn must be less than or equal to --return-fail.");
    }

    if (options.stepUserUnits > MaxRecommendedStepUserUnits && (options.tinyMotion || options.dualMotion || options.allMotion))
    {
        throw std::runtime_error(
            "Refusing --step greater than 0.25 user units for this starter demo. "
            "Use 0.05 or smaller for normal testing.");
    }

    if (options.jointVectorMotion)
    {
        if (!jointVectorHasMotion(options.jointVectorUserUnits))
        {
            throw std::runtime_error("--joints must contain at least one nonzero joint value for --joint-vector.");
        }

        if (maxAbsJointVector(options.jointVectorUserUnits) > MaxRecommendedJointVectorUserUnits)
        {
            throw std::runtime_error(
                "Refusing a joint-vector element greater than 0.25 user units for this starter demo. "
                "Use smaller values first.");
        }
    }
}
}

int main(int argc, char* argv[])
{
    const std::vector<std::string> args(argv + 1, argv + argc);

    if (hasArg(args, "--help") || hasArg(args, "-h"))
    {
        printUsage();
        return 0;
    }

    if (hasArg(args, "--session-server"))
    {
        return runSessionServer();
    }

    try
    {
        Racer3RunOptions options;
        options.dryRun = hasArg(args, "--dry-run");
        options.tinyMotion = hasArg(args, "--tiny-motion");
        options.dualMotion = hasArg(args, "--dual-motion");
        options.allMotion = hasArg(args, "--all-motion");
        options.jointVectorMotion = hasArg(args, "--joint-vector");
        options.robotModelProbe = hasArg(args, "--robot-model-probe");
        options.robotPoseProbe = hasArg(args, "--robot-pose-probe");
        options.kinematicsDryRun = hasArg(args, "--kinematics-dry-run");
        options.cartesianVectorMotion = hasArg(args, "--cartesian-vector");
        options.cartesianTraceMotion = hasArg(args, "--cartesian-trace");
        options.positionOnlyIk = hasArg(args, "--position-only");
        options.compactMotion = hasArg(args, "--compact-motion");
        options.appendMotion = hasArg(args, "--append-motion");
        options.trajectoryMotion = hasArg(args, "--trajectory-motion");
        options.endpointOnlyMotion = hasArg(args, "--endpoint-only");
        options.segmentGoalMotion = hasArg(args, "--segment-goal");
        options.enableOnly = hasArg(args, "--enable-only") || (!options.dryRun && !options.tinyMotion && !options.dualMotion && !options.allMotion && !options.jointVectorMotion && !options.robotModelProbe && !options.robotPoseProbe && !options.kinematicsDryRun && !options.cartesianVectorMotion && !options.cartesianTraceMotion);
        options.motionConfirmed = hasArg(args, "--confirm-motion");
        options.diagnostics = hasArg(args, "--diagnostics");
        options.stepUserUnits = getDoubleOption(args, "--step", DefaultStepUserUnits);
        options.velocityUserUnitsPerSecond = getDoubleOption(args, "--velocity", DefaultVelocityUserUnitsPerSecond);
        options.returnWarnToleranceUserUnits = getDoubleOption(args, "--return-warn", DefaultReturnWarnToleranceUserUnits);
        options.returnFailToleranceUserUnits = getDoubleOption(args, "--return-fail", DefaultReturnFailToleranceUserUnits);

        const std::string jointsText = getStringOption(args, "--joints", "");
        if (options.jointVectorMotion)
        {
            if (jointsText.empty())
            {
                throw std::runtime_error("--joint-vector requires --joints j1,j2,j3,j4,j5,j6.");
            }
            options.jointVectorUserUnits = parseJointVector(jointsText);
        }

        const std::string cartesianText = getStringOption(args, "--cartesian", "0,0,0,0,0,0");
        if (options.kinematicsDryRun || options.cartesianVectorMotion)
        {
            options.cartesianVector = parseCartesianVector(cartesianText);
        }

        const std::string cartesianWaypointsText = getStringOption(args, "--cartesian-waypoints", "");
        if (options.cartesianTraceMotion)
        {
            if (cartesianWaypointsText.empty())
            {
                throw std::runtime_error("--cartesian-trace requires --cartesian-waypoints \"dx,dy,dz,dr,dp,dy;...\".");
            }

            options.cartesianTraceWaypoints = parseCartesianWaypoints(cartesianWaypointsText);
        }

        if (hasArg(args, "--vel"))
        {
            options.velocityUserUnitsPerSecond = getDoubleOption(args, "--vel", DefaultVelocityUserUnitsPerSecond);
        }

        validateOptions(options);

        const int motionModeCount =
            (options.tinyMotion ? 1 : 0) +
            (options.dualMotion ? 1 : 0) +
            (options.allMotion ? 1 : 0) +
            (options.jointVectorMotion ? 1 : 0) +
            (options.robotModelProbe ? 1 : 0) +
            (options.robotPoseProbe ? 1 : 0) +
            (options.kinematicsDryRun ? 1 : 0) +
            (options.cartesianVectorMotion ? 1 : 0) +
            (options.cartesianTraceMotion ? 1 : 0);
        if (motionModeCount > 1)
        {
            std::cerr << "Use only one motion mode: --tiny-motion, --dual-motion, --all-motion, --joint-vector, --robot-model-probe, --robot-pose-probe, --kinematics-dry-run, --cartesian-vector, or --cartesian-trace.\n";
            return 2;
        }

        if ((options.tinyMotion || options.dualMotion || options.allMotion || options.jointVectorMotion) && !options.motionConfirmed && !options.dryRun)
        {
            std::cerr << "Refusing to run real motion without --confirm-motion.\n";
            std::cerr << "Use --dry-run first, then run the motion mode with --confirm-motion only when ready.\n";
            return 2;
        }

        std::cout << "Racer3 RMP Basic Motion Demo\n";
        std::cout << "Robot: Comau Racer 3\n";
        std::cout << "Controller/API: RSI RMP / RapidCode\n";

        if (options.dryRun)
        {
            std::cout << "Mode: DRY RUN - no controller connection, no amp enable, no motion.\n";
        }
        else if (options.enableOnly)
        {
            std::cout << "Mode: ENABLE ONLY - connect, clear faults, enable, wait, disable. No motion.\n";
        }
        else if (options.tinyMotion)
        {
            std::cout << "Mode: TINY MOTION - isolated Axis 6 / J6 relative motion.\n";
        }
        else if (options.dualMotion)
        {
            std::cout << "Mode: DUAL MOTION - synchronized Axis 5 / J5 and Axis 6 / J6 relative motion.\n";
        }
        else if (options.allMotion)
        {
            std::cout << "Mode: ALL MOTION - synchronized Axis 1 through Axis 6 relative motion.\n";
        }
        else if (options.jointVectorMotion)
        {
            std::cout << "Mode: JOINT VECTOR - custom synchronized Axis 1 through Axis 6 relative motion.\n";
        }
        else if (options.robotModelProbe)
        {
            std::cout << "Mode: ROBOT MODEL PROBE - no amp enable, no motion.\n";
        }
        else if (options.robotPoseProbe)
        {
            std::cout << "Mode: ROBOT POSE PROBE - no amp enable, no motion.\n";
        }
        else if (options.kinematicsDryRun)
        {
            std::cout << "Mode: KINEMATICS DRY RUN - custom OpenRAVE Racer3 FK scaffold, no amp enable, no motion.\n";
        }
        else if (options.cartesianVectorMotion)
        {
            if (options.motionConfirmed)
            {
                std::cout << "Mode: CARTESIAN VECTOR - guarded Cartesian IK candidate with motion enabled only if validation gates pass.\n";
            }
            else
            {
                std::cout << "Mode: CARTESIAN VECTOR DRY RUN - compute guarded Cartesian IK candidate only; no amp enable, no motion.\n";
            }
        }
        else if (options.cartesianTraceMotion)
        {
            if (options.motionConfirmed)
            {
                std::cout << "Mode: CARTESIAN TRACE - guarded multi-waypoint trace with motion enabled only if validation gates pass.\n";
            }
            else
            {
                std::cout << "Mode: CARTESIAN TRACE VALIDATION - compute guarded multi-waypoint IK plan only; no amp enable, no motion.\n";
            }
        }

        std::cout << "Motion step: " << options.stepUserUnits
                  << " user units = " << (options.stepUserUnits * 360.0) << " degrees.\n";
        std::cout << "Motion velocity: " << options.velocityUserUnitsPerSecond
                  << " user-units/sec = " << (options.velocityUserUnitsPerSecond * 360.0) << " deg/sec.\n";
        if (options.jointVectorMotion)
        {
            std::cout << "Joint vector [J1..J6] user units:";
            for (double value : options.jointVectorUserUnits)
            {
                std::cout << " " << value;
            }
            std::cout << "\n";
        }

        if (options.kinematicsDryRun || options.cartesianVectorMotion)
        {
            std::cout << "Cartesian delta [dx,dy,dz meters, droll,dpitch,dyaw radians]:";
            for (double value : options.cartesianVector)
            {
                std::cout << " " << value;
            }
            std::cout << "\n";
        }

        if (options.cartesianTraceMotion)
        {
            std::cout << "Cartesian trace waypoints [dx,dy,dz meters, droll,dpitch,dyaw radians]: "
                      << options.cartesianTraceWaypoints.size()
                      << "\n";
            for (size_t waypointIndex = 0; waypointIndex < options.cartesianTraceWaypoints.size(); ++waypointIndex)
            {
                std::cout << "  Waypoint " << (waypointIndex + 1) << ":";
                for (double value : options.cartesianTraceWaypoints[waypointIndex])
                {
                    std::cout << " " << value;
                }
                std::cout << "\n";
            }
        }

        if (options.cartesianVectorMotion || options.cartesianTraceMotion)
        {
            std::cout << "Position-only IK: " << (options.positionOnlyIk ? "ON" : "OFF") << "\n";
            std::cout << "Compact segmented motion: " << (options.compactMotion ? "ON" : "OFF") << "\n";
            std::cout << "Append queued motion: " << (options.appendMotion ? "ON" : "OFF") << "\n";
            std::cout << "PVT trajectory motion: " << (options.trajectoryMotion ? "ON" : "OFF") << "\n";
            std::cout << "Endpoint-only point motion: " << (options.endpointOnlyMotion ? "ON" : "OFF") << "\n";
            std::cout << "Segment-goal smooth motion: " << (options.segmentGoalMotion ? "ON" : "OFF") << "\n";
        }
        std::cout << "Diagnostics: " << (options.diagnostics ? "FULL" : "COMPACT") << "\n";
        std::cout << "Return warning tolerance: " << options.returnWarnToleranceUserUnits
                  << " user units = " << (options.returnWarnToleranceUserUnits * 360.0) << " degrees.\n";
        std::cout << "Return fail tolerance: " << options.returnFailToleranceUserUnits
                  << " user units = " << (options.returnFailToleranceUserUnits * 360.0) << " degrees.\n";
        std::cout << "WARNING: Real modes may enable robot drives. Keep the robot area clear and E-stop ready.\n";
        std::cout << "Press Enter to continue or Ctrl+C to abort...\n";
        std::string ignored;
        std::getline(std::cin, ignored);

        Racer3BasicMotion demo;
        demo.run(options);
        std::cout << "Racer3 test finished successfully.\n";
        return 0;
    }
    catch (const RSI::RapidCode::RsiError& error)
    {
        std::cerr << "RapidCode error: " << error.text << "\n";
        std::cerr << "Function: " << error.functionName << "\n";
        return 1;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Error: " << exception.what() << "\n";
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unknown error occurred.\n";
        return 1;
    }
}
