#include "Racer3BasicMotion.h"

#include <cstdlib>
#include <iostream>
#include <rsi.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr double DefaultStepUserUnits = 0.05;
constexpr double DefaultVelocityUserUnitsPerSecond = 0.05;
constexpr double MaxRecommendedStepUserUnits = 0.25;

void printUsage()
{
    std::cout
        << "Racer3 RMP Basic Motion Demo\n\n"
        << "Usage:\n"
        << "  racer3-basic-motion --dry-run [--step 0.05] [--velocity 0.05] [--diagnostics]\n"
        << "  racer3-basic-motion --enable-only [--diagnostics]\n"
        << "  racer3-basic-motion --tiny-motion --confirm-motion [--step 0.05] [--velocity 0.05] [--diagnostics]\n"
        << "  racer3-basic-motion --dual-motion --confirm-motion [--step 0.05] [--velocity 0.05] [--diagnostics]\n\n"
        << "Modes:\n"
        << "  --dry-run          Print the planned sequence only. No RMP connection.\n"
        << "  --enable-only      Connect, clear faults, enable, wait, disable. This is the default.\n"
        << "  --tiny-motion      Enable all drives, isolate J6, and run a tiny J6-only move.\n"
        << "  --dual-motion      Enable all drives, remap MultiAxis to J5+J6, and move both together.\n"
        << "  --confirm-motion   Required safety acknowledgement for any real motion.\n"
        << "  --diagnostics      Print full diagnostic dumps. Default output is compact.\n"
        << "  --step <value>     Relative move in user units. Default 0.05.\n"
        << "  --velocity <value> Velocity in user-units/sec. Default 0.05.\n"
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

    if (options.stepUserUnits > MaxRecommendedStepUserUnits && (options.tinyMotion || options.dualMotion))
    {
        throw std::runtime_error(
            "Refusing --step greater than 0.25 user units for this starter demo. "
            "Use 0.05 or smaller for normal testing.");
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

    try
    {
        Racer3RunOptions options;
        options.dryRun = hasArg(args, "--dry-run");
        options.tinyMotion = hasArg(args, "--tiny-motion");
        options.dualMotion = hasArg(args, "--dual-motion");
        options.enableOnly = hasArg(args, "--enable-only") || (!options.dryRun && !options.tinyMotion && !options.dualMotion);
        options.motionConfirmed = hasArg(args, "--confirm-motion");
        options.diagnostics = hasArg(args, "--diagnostics");
        options.stepUserUnits = getDoubleOption(args, "--step", DefaultStepUserUnits);
        options.velocityUserUnitsPerSecond = getDoubleOption(args, "--velocity", DefaultVelocityUserUnitsPerSecond);

        if (hasArg(args, "--vel"))
        {
            options.velocityUserUnitsPerSecond = getDoubleOption(args, "--vel", DefaultVelocityUserUnitsPerSecond);
        }

        validateOptions(options);

        if (options.tinyMotion && options.dualMotion)
        {
            std::cerr << "Use only one motion mode: --tiny-motion or --dual-motion.\n";
            return 2;
        }

        if ((options.tinyMotion || options.dualMotion) && !options.motionConfirmed && !options.dryRun)
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

        std::cout << "Motion step: " << options.stepUserUnits
                  << " user units = " << (options.stepUserUnits * 360.0) << " degrees.\n";
        std::cout << "Motion velocity: " << options.velocityUserUnitsPerSecond
                  << " user-units/sec = " << (options.velocityUserUnitsPerSecond * 360.0) << " deg/sec.\n";
        std::cout << "Diagnostics: " << (options.diagnostics ? "FULL" : "COMPACT") << "\n";
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
