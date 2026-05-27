#include "Racer3BasicMotion.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <chrono>
#include <thread>
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
constexpr double DefaultAxis6JogVelocityUserUnitsPerSecond = 0.005;
constexpr double MaxAxis6JogVelocityUserUnitsPerSecond = 0.010;
constexpr double DefaultCartesianJogSpeedMetersPerSecond = 0.003;
constexpr double MaxCartesianJogSpeedMetersPerSecond = 0.004;
constexpr double MaxRecommendedStepUserUnits = 0.25;
constexpr int AxisCount = 6;
constexpr double MaxRecommendedJointVectorUserUnits = 0.25;
constexpr double DefaultKeyboardJogVelocityUserUnitsPerSecond = 0.002;
constexpr double DefaultKeyboardJogLoopPeriodMilliseconds = 20.0;
constexpr double DefaultCartesianKeyboardJogSpeedMetersPerSecond = 0.003;
constexpr double DefaultCartesianKeyboardJogAngularSpeedRadiansPerSecond = 0.15;

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
        << "  racer3-basic-motion --session-server\n"
        << "  racer3-basic-motion --prearm-hold [--velocity 0.002] [--diagnostics]\n"
        << "  racer3-basic-motion --keyboard-jog-endpoint-only --jog-axis 6 [--velocity 0.002] [--confirm-keyboard-jog] [--diagnostics]\n"
        << "  racer3-basic-motion --keyboard-cartesian-jog-endpoint-only [--xbox-controller] [--cartesian-jog-linear-speed 0.006] [--keyboard-base-rotate-speed 0.015] [--cartesian-jog-gain-x 1.0] [--cartesian-jog-gain-z 0.65] [--cartesian-jog-max-joint-velocity 0.030] [--confirm-keyboard-cartesian-jog] [--diagnostics]\n\n"
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
        << "  --session-server Start a persistent local armed session. Connects RMP, enables amps once, accepts status/stop/shutdown, trace, backend-owned Axis 6 velocity diagnostics, backend-owned Cartesian X+/Z- jog commands, and no-motion RTTask probe commands.\n"
        << "  --prearm-hold Start the same bottom-to-top all-axis pre-arm/enable path, print prearm_ready, hold amps enabled, and wait for shutdown on stdin. Used by RTTask jog proof.\n"
        << "  --keyboard-jog-endpoint-only Start the bottom-to-top all-axis pre-arm path, then run a local C++ keyboard loop for endpoint-only Axis 6/J6 jog pulses.\n"
        << "  --keyboard-cartesian-jog-endpoint-only Start the bottom-to-top all-axis pre-arm path, then run a local C++ operator-friendly keyboard loop: W/S endpoint X, R/F endpoint Z, A/D direct base rotate, H home, Q/Esc exit.\n"
        << "  --xbox-controller Also poll XInput slot 0 for an Xbox 360-compatible controller during --keyboard-cartesian-jog-endpoint-only. Left stick Y=X reach, left stick X=base, right stick Y=Z, Y=H-home, B/Back=exit. Keyboard stays active.\n"
        << "  --position-only   For --cartesian-vector, solve and validate only XYZ position. Roll/pitch/yaw residual is printed but not gated.\n"
        << "  --compact-motion For --cartesian-vector confirmed segmented motion, skip per-segment live samples/status dumps.\n"
        << "  --append-motion  Experimental: queue segmented MoveRelative commands with APPEND.\n"
        << "  --trajectory-motion Experimental: stream validated segment endpoints with MultiAxis::MovePVT.\n"
        << "  --endpoint-only  Experimental: solve one final XYZ target and run smooth joint-space PVT. Not a straight Cartesian TCP path.\n"
        << "  --segment-goal  Experimental: use segmented IK only to find final goal, then run smooth joint-space PVT.\n"
        << "  --confirm-motion   Required safety acknowledgement for any real motion.\n"
        << "  --confirm-keyboard-jog Required before --keyboard-jog-endpoint-only will issue jog pulses. Without it, keys are observed but motion is blocked.\n"
        << "  --confirm-keyboard-cartesian-jog Required before --keyboard-cartesian-jog-endpoint-only will issue Cartesian jog spans. Without it, keys are observed but motion is blocked.\n"
        << "  --jog-axis <1-6> Operator-facing axis for --keyboard-jog-endpoint-only. First implementation supports Axis 6/J6 only.\n"
        << "  --cartesian-jog-speed <value> Backward-compatible scalar for both linear and angular Cartesian keyboard jog speeds. Default 0.003.\n"
        << "  --cartesian-jog-linear-speed <value> Endpoint keyboard jog linear speed in meters/sec for X/Z. Overrides --cartesian-jog-speed for X/Z. Default 0.003.\n"
        << "  --cartesian-jog-angular-speed <value> Cartesian keyboard jog angular speed in radians/sec for roll/pitch/yaw. Overrides --cartesian-jog-speed for RPY. Default 0.10.\n"
        << "  --cartesian-jog-gain-x <value> Multiplier for X jog target speed. Default 1.0.\n"
        << "  --cartesian-jog-gain-y <value> Legacy/unused for operator-friendly keyboard mode; A/D now use direct base rotate. Default 1.25.\n"
        << "  --keyboard-base-rotate-speed <value> Direct J1/base rotate speed for A/D in user-units/sec. Default 0.015.\n"
        << "  --cartesian-jog-gain-z <value> Multiplier for Z jog target speed. Default 0.65.\n"
        << "  --cartesian-jog-max-joint-velocity <value> Max absolute joint velocity for Cartesian keyboard jog in user-units/sec. Default 0.030.\n"
        << "  --keyboard-jog-period-ms <value> Pulse loop period for endpoint-only keyboard jog. Default 20 ms.\n"
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
        writeSessionEvent("{\"type\":\"session_ready\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Persistent armed session ready. Amps are enabled and will stay enabled until Shutdown Session. Trace commands, backend Axis 6 velocity jog proof commands, backend-owned Cartesian X+/Z- jog-loop commands, and no-motion RTTask probe commands are available.\"}");
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

        if (command.find("rttask_probe_start") != std::string::npos)
        {
            const std::string libraryDirectory = jsonStringField(line, "libraryDirectory").empty()
                ? jsonStringField(line, "libraryDir")
                : jsonStringField(line, "libraryDirectory");
            const std::string rttaskDirectory = jsonStringField(line, "rttaskDirectory").empty()
                ? jsonStringField(line, "rtTaskDirectory")
                : jsonStringField(line, "rttaskDirectory");
            std::string managerPlatform = jsonStringField(line, "managerPlatform").empty()
                ? jsonStringField(line, "platform")
                : jsonStringField(line, "managerPlatform");
            const int statusPeriodMilliseconds = static_cast<int>(jsonNumberField(line, "statusPeriodMs", 10.0));
            const int intentPeriodMilliseconds = static_cast<int>(jsonNumberField(line, "intentPeriodMs", 10.0));

            try
            {
                writeSessionEvent("{\"type\":\"session_rttask_probe_starting\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Starting no-motion Racer3 RTTask heartbeat/status probe. This does not command motion.\"}");
                sessionMotion.startArmedSessionRtTaskProbe(libraryDirectory, rttaskDirectory, managerPlatform, statusPeriodMilliseconds, intentPeriodMilliseconds);
                writeSessionEvent(sessionMotion.getArmedSessionRtTaskProbeStatusJson());
            }
            catch (const RR::RsiError& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_rttask_probe_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                    (sessionMotion.areArmedSessionAmpsEnabled() ? "true" : "false") +
                    ",\"message\":\"RTTask probe RapidCode error: " + escapeJsonText(error.text) + " (" + escapeJsonText(error.functionName) + ").\"}");
            }
            catch (const std::exception& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_rttask_probe_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                    (sessionMotion.areArmedSessionAmpsEnabled() ? "true" : "false") +
                    ",\"message\":\"RTTask probe failed: " + escapeJsonText(error.what()) + ".\"}");
            }

            continue;
        }

        if (command.find("rttask_probe_status") != std::string::npos)
        {
            try
            {
                writeSessionEvent(sessionMotion.getArmedSessionRtTaskProbeStatusJson());
            }
            catch (const RR::RsiError& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_rttask_probe_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                    (sessionMotion.areArmedSessionAmpsEnabled() ? "true" : "false") +
                    ",\"message\":\"RTTask probe status RapidCode error: " + escapeJsonText(error.text) + " (" + escapeJsonText(error.functionName) + ").\"}");
            }
            catch (const std::exception& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_rttask_probe_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                    (sessionMotion.areArmedSessionAmpsEnabled() ? "true" : "false") +
                    ",\"message\":\"RTTask probe status failed: " + escapeJsonText(error.what()) + ".\"}");
            }

            continue;
        }

        if (command.find("rttask_probe_stop") != std::string::npos)
        {
            sessionMotion.stopArmedSessionRtTaskProbe();
            writeSessionEvent(std::string("{\"type\":\"session_rttask_probe_stopped\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                (sessionMotion.areArmedSessionAmpsEnabled() ? "true" : "false") +
                ",\"message\":\"Racer3 RTTask probe stopped. No motion commands were issued.\"}");
            continue;
        }

        if (command.find("status") != std::string::npos || command.find("hello") != std::string::npos)
        {
            sessionMotion.printArmedSessionPositionSnapshot("Session status position snapshot");
            const bool ampsEnabled = sessionMotion.areArmedSessionAmpsEnabled();
            writeSessionEvent(std::string("{\"type\":\"session_status\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                (ampsEnabled ? "true" : "false") +
                ",\"message\":\"Persistent armed session is alive. Position snapshot was printed to backend stdout. Trace commands, backend Axis 6 velocity jog proof commands, backend-owned Cartesian X+/Z- jog-loop commands, and no-motion RTTask probe commands are available.\"}");
            continue;
        }

        if (command.find("jog_cartesian_start") != std::string::npos)
        {
            const std::string requestedAxis = normalizeSessionCommand(jsonStringField(line, "axis"));
            const std::string direction = normalizeSessionCommand(jsonStringField(line, "direction"));
            const double requestedSpeed = jsonNumberField(
                line,
                "speedMetersPerSecond",
                jsonNumberField(line, "speed", DefaultCartesianJogSpeedMetersPerSecond));

            try
            {
                if (requestedSpeed <= 0.0 || requestedSpeed > MaxCartesianJogSpeedMetersPerSecond)
                {
                    std::ostringstream message;
                    message << "backend Cartesian jog requires 0 < speed <= "
                            << MaxCartesianJogSpeedMetersPerSecond
                            << " meters/sec.";
                    throw std::runtime_error(message.str());
                }

                const bool xPositive =
                    direction == "x+" ||
                    direction == "+x" ||
                    direction == "positive_x" ||
                    direction == "xpositive" ||
                    (requestedAxis == "x" && (direction.empty() || direction == "+" || direction == "positive"));

                const bool zNegative =
                    direction == "z-" ||
                    direction == "-z" ||
                    direction == "negative_z" ||
                    direction == "znegative" ||
                    (requestedAxis == "z" && (direction == "-" || direction == "negative"));

                if (!xPositive && !zNegative)
                {
                    throw std::runtime_error("backend Cartesian jog v2 accepts X+ and Z-. Recommended first test: {\"type\":\"jog_cartesian_start\",\"direction\":\"Z-\",\"speed\":0.003}.");
                }

                std::array<double, AxisCount> cartesianDirection{};
                const char* directionLabel = xPositive ? "X+" : "Z-";
                if (xPositive)
                {
                    cartesianDirection[0] = 1.0;
                }
                else
                {
                    cartesianDirection[2] = -1.0;
                }

                writeSessionEvent(std::string("{\"type\":\"session_jog_cartesian_starting\",\"state\":\"jog_running\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Backend Cartesian ") + directionLabel + " jog command accepted. Backend-owned endpoint-only IK jog loop will generate motion until stop.\"}");
                sessionMotion.startArmedSessionCartesianJog(cartesianDirection, requestedSpeed);
                const bool ampsEnabledAfterStart = sessionMotion.areArmedSessionAmpsEnabled();
                writeSessionEvent(std::string("{\"type\":\"session_jog_cartesian_started\",\"state\":\"jog_running\",\"armed\":true,\"ampsEnabled\":") +
                    (ampsEnabledAfterStart ? "true" : "false") +
                    ",\"message\":\"Backend-owned Cartesian " + directionLabel + " jog loop is active. Send jog_cartesian_stop to stop/decelerate and keep amps enabled.\"}");
            }
            catch (const RR::RsiError& error)
            {
                const bool ampsEnabled = sessionMotion.areArmedSessionAmpsEnabled();
                writeSessionEvent(std::string("{\"type\":\"session_jog_cartesian_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                    (ampsEnabled ? "true" : "false") +
                    ",\"message\":\"Cartesian jog RapidCode error: " + escapeJsonText(error.text) + " (" + escapeJsonText(error.functionName) + "). Check backend snapshot before sending another jog start.\"}");
            }
            catch (const std::exception& error)
            {
                const bool ampsEnabled = sessionMotion.areArmedSessionAmpsEnabled();
                writeSessionEvent(std::string("{\"type\":\"session_jog_cartesian_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                    (ampsEnabled ? "true" : "false") +
                    ",\"message\":\"Cartesian jog failed: " + escapeJsonText(error.what()) + ". Check backend snapshot before sending another jog start.\"}");
            }

            continue;
        }

        if (command.find("jog_cartesian_stop") != std::string::npos)
        {
            try
            {
                writeSessionEvent("{\"type\":\"session_jog_cartesian_stopping\",\"state\":\"jog_stopping\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Backend Cartesian jog stop requested. v14 waits for the current short non-append PVT smoothing span to finish naturally; no Abort, TriggeredModify, APPEND, or amp disable on normal release.\"}");
                sessionMotion.stopArmedSessionCartesianJog("jog_cartesian_stop command");
                const bool ampsEnabledAfterStop = sessionMotion.areArmedSessionAmpsEnabled();
                writeSessionEvent(std::string("{\"type\":\"session_jog_cartesian_stopped\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                    (ampsEnabledAfterStop ? "true" : "false") +
                    ",\"message\":\"Backend Cartesian jog stop completed MotionDoneWait/settle check. Check ampsEnabled and backend snapshot before additional jog commands.\"}");
            }
            catch (const RR::RsiError& error)
            {
                const bool ampsEnabled = sessionMotion.areArmedSessionAmpsEnabled();
                writeSessionEvent(std::string("{\"type\":\"session_jog_cartesian_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                    (ampsEnabled ? "true" : "false") +
                    ",\"message\":\"Cartesian jog stop RapidCode error: " + escapeJsonText(error.text) + " (" + escapeJsonText(error.functionName) + "). Check backend snapshot before additional motion.\"}");
            }
            catch (const std::exception& error)
            {
                const bool ampsEnabled = sessionMotion.areArmedSessionAmpsEnabled();
                writeSessionEvent(std::string("{\"type\":\"session_jog_cartesian_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                    (ampsEnabled ? "true" : "false") +
                    ",\"message\":\"Cartesian jog stop failed: " + escapeJsonText(error.what()) + ". Check backend snapshot before additional motion.\"}");
            }

            continue;
        }

        if (command.find("jog_velocity_start") != std::string::npos)
        {
            const int requestedAxis = static_cast<int>(jsonNumberField(line, "axis", 6.0));
            const std::string direction = normalizeSessionCommand(jsonStringField(line, "direction"));
            const double requestedVelocity = jsonNumberField(line, "velocity", DefaultAxis6JogVelocityUserUnitsPerSecond);

            try
            {
                if (requestedAxis != 6)
                {
                    throw std::runtime_error("backend velocity jog proof currently accepts only axis=6 / J6.");
                }

                if (requestedVelocity <= 0.0 || requestedVelocity > MaxAxis6JogVelocityUserUnitsPerSecond)
                {
                    std::ostringstream message;
                    message << "backend Axis 6 velocity jog requires 0 < velocity <= "
                            << MaxAxis6JogVelocityUserUnitsPerSecond
                            << " user-units/sec.";
                    throw std::runtime_error(message.str());
                }

                double signedVelocity = requestedVelocity;
                if (direction == "negative" || direction == "-" || direction == "j6-" || direction == "axis6-" || direction == "cw")
                {
                    signedVelocity = -requestedVelocity;
                }
                else if (!(direction.empty() || direction == "positive" || direction == "+" || direction == "j6+" || direction == "axis6+" || direction == "ccw"))
                {
                    throw std::runtime_error("backend Axis 6 velocity jog direction must be positive/J6+ or negative/J6-.");
                }

                writeSessionEvent("{\"type\":\"session_jog_velocity_starting\",\"state\":\"jog_running\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Backend Axis 6 velocity jog command accepted. This is a joint-space smooth-motion proof, not Cartesian TCP jog yet.\"}");
                sessionMotion.startArmedSessionAxis6VelocityJog(signedVelocity);
                writeSessionEvent("{\"type\":\"session_jog_velocity_started\",\"state\":\"jog_running\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Backend Axis 6 velocity jog is active. Send jog_velocity_stop to decelerate to zero velocity.\"}");
            }
            catch (const RR::RsiError& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_jog_velocity_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Axis 6 velocity jog RapidCode error: ") + escapeJsonText(error.text) + " (" + escapeJsonText(error.functionName) + "). Amps should remain enabled; use jog_velocity_stop or Shutdown Session if needed.\"}");
            }
            catch (const std::exception& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_jog_velocity_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Axis 6 velocity jog failed: ") + escapeJsonText(error.what()) + ". Amps should remain enabled; use jog_velocity_stop or Shutdown Session if needed.\"}");
            }

            continue;
        }

        if (command.find("jog_velocity_stop") != std::string::npos)
        {
            try
            {
                writeSessionEvent("{\"type\":\"session_jog_velocity_stopping\",\"state\":\"jog_stopping\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Backend Axis 6 velocity jog stop requested. Using TriggeredModify, not Abort or amp disable.\"}");
                sessionMotion.stopArmedSessionAxis6VelocityJog("jog_velocity_stop command");
                writeSessionEvent("{\"type\":\"session_jog_velocity_stopped\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Backend Axis 6 velocity jog stop completed MotionDoneWait/settle check. Amps remain enabled for additional session commands.\"}");
            }
            catch (const RR::RsiError& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_jog_velocity_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Axis 6 velocity jog stop RapidCode error: ") + escapeJsonText(error.text) + " (" + escapeJsonText(error.functionName) + "). Amps should remain enabled; use Shutdown Session if needed.\"}");
            }
            catch (const std::exception& error)
            {
                writeSessionEvent(std::string("{\"type\":\"session_jog_velocity_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Axis 6 velocity jog stop failed: ") + escapeJsonText(error.what()) + ". Amps should remain enabled; use Shutdown Session if needed.\"}");
            }

            continue;
        }

        if (command.find("stop") != std::string::npos)
        {
            try
            {
                sessionMotion.stopArmedSessionMotion();
                writeSessionEvent("{\"type\":\"session_stop_ack\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Stop acknowledged. Active backend Cartesian or Axis 6 velocity jog is decelerated with TriggeredModify; otherwise legacy trace/general motion stop uses Abort. Amps remain enabled for the armed session.\"}");
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
            writeSessionEvent("{\"type\":\"session_reject\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Unsupported jog command. Supported backend jog commands: jog_cartesian_start / jog_cartesian_stop for the backend-owned Cartesian X+/Z- jog loop, jog_velocity_start / jog_velocity_stop for Axis 6 diagnostic proof motion, and rttask_probe_start/status/stop for no-motion RTTask validation.\"}");
            continue;
        }

        writeSessionEvent("{\"type\":\"session_error\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Unknown armed session command. Supported now: hello, status, trace, rttask_probe_start/status/stop, jog_cartesian_start, jog_cartesian_stop, jog_velocity_start, jog_velocity_stop, stop, shutdown.\"}");
    }

    writeSessionEvent("{\"type\":\"session_input_closed\",\"state\":\"shutting_down\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Session input closed. Disabling amps and exiting.\"}");
    sessionMotion.shutdownArmedSession();
    return 0;
}


int runPrearmHold(const std::vector<std::string>& args)
{
    const double velocityUserUnitsPerSecond = getDoubleOption(args, "--velocity", 0.002);
    const double prearmHoldSeconds = getDoubleOption(args, "--prearm-hold-seconds", 0.0);
    const bool diagnostics = hasArg(args, "--diagnostics");

    if (velocityUserUnitsPerSecond <= 0.0)
    {
        writeSessionEvent("{\"type\":\"prearm_error\",\"state\":\"faulted\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"--velocity must be greater than zero.\"}");
        return 2;
    }

    writeSessionEvent("{\"type\":\"prearm_starting\",\"state\":\"starting\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Bottom-to-top endpoint pre-arm hold starting. This clears faults, relaxes startup Home/ErrorLimit actions, enables Axis 1 through Axis 6, then holds amps enabled until shutdown.\"}");

    Racer3BasicMotion prearmMotion;

    try
    {
        prearmMotion.startArmedSession(velocityUserUnitsPerSecond, diagnostics);
        const bool ampsEnabled = prearmMotion.areArmedSessionAmpsEnabled();
        if (!ampsEnabled)
        {
            throw std::runtime_error("Pre-arm hold completed startup but amp verification is false.");
        }

        writeSessionEvent("{\"type\":\"prearm_ready\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Bottom-to-top endpoint pre-arm hold is ready. All axes and MultiAxis are expected to remain amp-enabled until shutdown. RTTask jog proof may now run.\"}");

        if (prearmHoldSeconds > 0.0)
        {
            writeSessionEvent("{\"type\":\"prearm_timed_hold\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Timed pre-arm hold active. Amps remain enabled until the hold timer expires.\"}");
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(prearmHoldSeconds * 1000.0)));
            writeSessionEvent("{\"type\":\"prearm_timed_shutdown_starting\",\"state\":\"shutting_down\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Timed pre-arm hold expired. Disabling amps and releasing controller.\"}");
            prearmMotion.shutdownArmedSession();
            writeSessionEvent("{\"type\":\"prearm_shutdown\",\"state\":\"exiting\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Bottom-to-top endpoint pre-arm hold shut down after timed hold. Amps disabled and controller released.\"}");
            return 0;
        }
    }
    catch (const RR::RsiError& error)
    {
        writeSessionEvent(std::string("{\"type\":\"prearm_error\",\"state\":\"faulted\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Failed to start pre-arm hold RapidCode error: ") + escapeJsonText(error.text) + " (" + escapeJsonText(error.functionName) + "). Shutdown attempted.\"}");
        prearmMotion.shutdownArmedSession();
        return 1;
    }
    catch (const std::exception& error)
    {
        writeSessionEvent(std::string("{\"type\":\"prearm_error\",\"state\":\"faulted\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Failed to start pre-arm hold: ") + escapeJsonText(error.what()) + ". Shutdown attempted.\"}");
        prearmMotion.shutdownArmedSession();
        return 1;
    }
    catch (...)
    {
        writeSessionEvent("{\"type\":\"prearm_error\",\"state\":\"faulted\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Failed to start pre-arm hold with unknown exception. Shutdown attempted.\"}");
        prearmMotion.shutdownArmedSession();
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
            writeSessionEvent("{\"type\":\"prearm_shutdown_starting\",\"state\":\"shutting_down\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Shutdown requested. Disabling amps and releasing controller.\"}");
            prearmMotion.shutdownArmedSession();
            writeSessionEvent("{\"type\":\"prearm_shutdown\",\"state\":\"exiting\",\"armed\":false,\"ampsEnabled\":false,\"message\":\"Bottom-to-top endpoint pre-arm hold shut down. Amps disabled and controller released.\"}");
            return 0;
        }

        if (command.find("status") != std::string::npos || command.find("hello") != std::string::npos)
        {
            prearmMotion.printArmedSessionPositionSnapshot("Pre-arm hold status position snapshot");
            const bool ampsEnabled = prearmMotion.areArmedSessionAmpsEnabled();
            writeSessionEvent(std::string("{\"type\":\"prearm_status\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":") +
                (ampsEnabled ? "true" : "false") +
                ",\"message\":\"Bottom-to-top endpoint pre-arm hold is alive. Amps remain enabled until shutdown.\"}");
            continue;
        }

        writeSessionEvent("{\"type\":\"prearm_reject\",\"state\":\"armed_idle\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Unsupported pre-arm hold command. Supported commands: hello, status, shutdown.\"}");
    }

    writeSessionEvent("{\"type\":\"prearm_input_closed\",\"state\":\"shutting_down\",\"armed\":true,\"ampsEnabled\":true,\"message\":\"Pre-arm hold input closed. Disabling amps and exiting.\"}");
    prearmMotion.shutdownArmedSession();
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

    if (hasArg(args, "--prearm-hold"))
    {
        return runPrearmHold(args);
    }

    if (hasArg(args, "--keyboard-cartesian-jog-endpoint-only"))
    {
        try
        {
            const double legacySpeed = getDoubleOption(args, "--cartesian-jog-speed", DefaultCartesianKeyboardJogSpeedMetersPerSecond);
            const double linearSpeed = getDoubleOption(args, "--cartesian-jog-linear-speed", legacySpeed);
            const double angularSpeed = getDoubleOption(args, "--cartesian-jog-angular-speed", 0.10);
            const double gainX = getDoubleOption(args, "--cartesian-jog-gain-x", 1.0);
            const double gainY = getDoubleOption(args, "--cartesian-jog-gain-y", 1.25);
            const double gainZ = getDoubleOption(args, "--cartesian-jog-gain-z", 0.65);
            const double maxJointVelocity = getDoubleOption(args, "--cartesian-jog-max-joint-velocity", 0.030);
            const double baseRotateSpeed = getDoubleOption(args, "--keyboard-base-rotate-speed", 0.015);
            const double loopPeriodMs = getDoubleOption(args, "--keyboard-jog-period-ms", DefaultKeyboardJogLoopPeriodMilliseconds);
            const bool confirmed = hasArg(args, "--confirm-keyboard-cartesian-jog");
            const bool diagnostics = hasArg(args, "--diagnostics");
            const bool xboxControllerEnabled = hasArg(args, "--xbox-controller") || hasArg(args, "--xbox-cartesian-jog");

            const double startupDelaySeconds = getDoubleOption(args, "--keyboard-startup-delay-seconds", 8.0);
            if (startupDelaySeconds > 0.0)
            {
                std::cout << "Cartesian keyboard jog startup delay: waiting "
                          << startupDelaySeconds
                          << " seconds for RMP shared memory/status to settle after rsiconfig.\n";
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(startupDelaySeconds * 1000.0)));
            }

            Racer3BasicMotion motion;
            motion.runEndpointOnlyCartesianKeyboardJog(
                linearSpeed,
                angularSpeed,
                loopPeriodMs / 1000.0,
                confirmed,
                diagnostics,
                gainX,
                gainY,
                gainZ,
                maxJointVelocity,
                baseRotateSpeed,
                xboxControllerEnabled);
            return 0;
        }
        catch (const RR::RsiError& error)
        {
            std::cerr << "RapidCode error: "
                      << error.text
                      << " ("
                      << error.functionName
                      << ")"
                      << "\n";
            return 1;
        }
        catch (const std::exception& error)
        {
            std::cerr << "Error: " << error.what() << "\n";
            return 1;
        }
    }

    if (hasArg(args, "--keyboard-jog-endpoint-only"))
    {
        try
        {
            const int operatorAxis = static_cast<int>(getDoubleOption(args, "--jog-axis", 6.0));
            const double velocity = getDoubleOption(args, "--velocity", DefaultKeyboardJogVelocityUserUnitsPerSecond);
            const double loopPeriodMs = getDoubleOption(args, "--keyboard-jog-period-ms", DefaultKeyboardJogLoopPeriodMilliseconds);
            const bool confirmed = hasArg(args, "--confirm-keyboard-jog");
            const bool diagnostics = hasArg(args, "--diagnostics");

            const double startupDelaySeconds = getDoubleOption(args, "--keyboard-startup-delay-seconds", 8.0);
            if (startupDelaySeconds > 0.0)
            {
                std::cout << "Keyboard jog startup delay: waiting "
                          << startupDelaySeconds
                          << " seconds for RMP shared memory/status to settle after rsiconfig.\n";
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(startupDelaySeconds * 1000.0)));
            }
            
            Racer3BasicMotion motion;
            motion.runEndpointOnlyKeyboardJog(
                operatorAxis,
                velocity,
                loopPeriodMs / 1000.0,
                confirmed,
                diagnostics);
            return 0;
        }
        catch (const RR::RsiError& error)
        {
            std::cerr << "RapidCode error: " << error.text << " (" << error.functionName << ")\n";
            return 1;
        }
        catch (const std::exception& error)
        {
            std::cerr << "Error: " << error.what() << "\n";
            return 1;
        }
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


