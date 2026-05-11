#include "Racer3BasicMotion.h"

#include <iostream>
#include <rsi.h>
#include <string>
#include <vector>

namespace
{
void printUsage()
{
    std::cout
        << "Racer3 RMP Basic Motion Demo\n\n"
        << "Usage:\n"
        << "  racer3-basic-motion --dry-run\n"
        << "  racer3-basic-motion --enable-only\n"
        << "  racer3-basic-motion --tiny-motion --confirm-motion\n\n"
        << "Modes:\n"
        << "  --dry-run          Print the planned sequence only. No RMP connection.\n"
        << "  --enable-only      Connect, clear faults, enable amps, wait briefly, disable amps. This is the default.\n"
        << "  --tiny-motion      Run a very small relative joint-space motion sequence. Requires --confirm-motion.\n"
        << "  --confirm-motion   Required safety acknowledgement for any real motion.\n"
        << "  --help             Show this help text.\n";
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
}

int main(int argc, char* argv[])
{
    const std::vector<std::string> args(argv + 1, argv + argc);

    if (hasArg(args, "--help") || hasArg(args, "-h"))
    {
        printUsage();
        return 0;
    }

    Racer3RunOptions options;
    options.dryRun = hasArg(args, "--dry-run");
    options.tinyMotion = hasArg(args, "--tiny-motion");
    options.enableOnly = hasArg(args, "--enable-only") || (!options.dryRun && !options.tinyMotion);
    options.motionConfirmed = hasArg(args, "--confirm-motion");

    if (options.tinyMotion && !options.motionConfirmed && !options.dryRun)
    {
        std::cerr << "Refusing to run real motion without --confirm-motion.\n";
        std::cerr << "Use --dry-run first, then run --tiny-motion --confirm-motion only when ready.\n";
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
        std::cout << "Mode: TINY MOTION - extremely small relative joint-space motion.\n";
    }

    std::cout << "WARNING: Real modes may enable robot drives. Keep the robot area clear and E-stop ready.\n";
    std::cout << "Press Enter to continue or Ctrl+C to abort...\n";
    std::string ignored;
    std::getline(std::cin, ignored);

    try
    {
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
