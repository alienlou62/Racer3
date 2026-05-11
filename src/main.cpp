#include "Racer3BasicMotion.h"
#include <iostream>
#include <rsi.h>
#include <string>

int main()
{
    std::cout << "Racer3 RMP Basic Motion Demo\n";
    std::cout << "WARNING: This program may move the robot. Keep the robot area clear and the E-stop ready.\n";
    std::cout << "Press Enter to continue or Ctrl+C to abort...\n";
    std::string ignored;
    std::getline(std::cin, ignored);

    try
    {
        Racer3BasicMotion demo;
        demo.run();
        std::cout << "Demo finished successfully.\n";
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
