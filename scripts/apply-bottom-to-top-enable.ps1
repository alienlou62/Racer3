param(
    [string]$Path = ".\src\Racer3BasicMotion.cpp"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $Path)) {
    throw "Could not find $Path. Run this from the repo root or pass -Path."
}

$source = Get-Content -Raw -Path $Path

$pattern = '(?s)void Racer3BasicMotion::enableAmplifiers\(\)\s*\{.*?\n\}\s*\n\s*void Racer3BasicMotion::enableOnlyTest\(\)'

$replacement = @'
void Racer3BasicMotion::enableAmplifiers()
{
    std::cout << "Enabling amplifiers bottom-to-top through individual Axis 1..6 objects, then verifying MultiAxis 6...\n";

    if (!multiAxis_)
    {
        throw std::runtime_error("MultiAxis object is not initialized.");
    }

    for (int index = 0; index < AxisCount; ++index)
    {
        if (!axes_[index])
        {
            throw std::runtime_error("Axis object " + std::to_string(index + 1) + " is not initialized.");
        }
    }

    std::cout << "Using overrideRestrictedState="
              << (OverrideRestrictedStateForEnable ? "true" : "false")
              << ".\n";

    std::cout << "Clearing MultiAxis 6 faults before ordered individual axis enable...\n";
    multiAxis_->ClearFaults();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    for (int index = 0; index < AxisCount; ++index)
    {
        std::cout << "Clearing faults and enabling hardware Axis "
                  << index
                  << " / RapidCode Axis "
                  << (index + 1)
                  << "...\n";

        try
        {
            axes_[index]->ClearFaults();
            std::this_thread::sleep_for(std::chrono::milliseconds(150));

            const int result = axes_[index]->AmpEnableSet(
                true,
                AmpEnableTimeoutMs,
                OverrideRestrictedStateForEnable);

            if (!axes_[index]->AmpEnableGet())
            {
                printAllAxisMotionStatus("All-axis status after individual axis AmpEnableGet false");
                throw std::runtime_error(
                    "Axis " + std::to_string(index + 1) +
                    " AmpEnableSet returned but AmpEnableGet is false.");
            }

            std::cout << "  Axis "
                      << (index + 1)
                      << " amp enable confirmed. AmpEnableSet returned "
                      << result
                      << " ms.\n";
        }
        catch (const RR::RsiError& error)
        {
            std::cout << "  Axis "
                      << (index + 1)
                      << " amp enable threw RapidCode error: "
                      << error.text
                      << " ("
                      << error.functionName
                      << ")\n";
            printAllAxisMotionStatus("All-axis status after individual axis AmpEnableSet exception");
            throw;
        }

        // Give the fieldbus/drive state a short chance to settle before enabling
        // the next drive. This is intentionally conservative because intermittent
        // AMP_ACTIVE timeouts were observed when enabling the whole MultiAxis group at once.
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    std::cout << "All individual axes report amp enabled. Verifying MultiAxis 6 amp-active state...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    try
    {
        const int result = multiAxis_->AmpEnableSet(
            true,
            AmpEnableTimeoutMs,
            OverrideRestrictedStateForEnable);

        if (!multiAxis_->AmpEnableGet())
        {
            printAllAxisMotionStatus("All-axis status after MultiAxis AmpEnableGet false");
            throw std::runtime_error("MultiAxis 6 AmpEnableSet returned but AmpEnableGet is false.");
        }

        std::cout << "MultiAxis 6 amp enable confirmed after ordered individual enable. AmpEnableSet returned "
                  << result
                  << " ms. Waiting briefly for drives to settle...\n";
    }
    catch (const RR::RsiError& error)
    {
        std::cout << "MultiAxis 6 amp enable verification threw RapidCode error after individual axes were enabled: "
                  << error.text
                  << " ("
                  << error.functionName
                  << ")\n";
        printAllAxisMotionStatus("All-axis status after MultiAxis verification exception");
        throw;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(EnableSettleMs));
}

void Racer3BasicMotion::enableOnlyTest()
'@

$newSource = [regex]::Replace($source, $pattern, $replacement, 1)

if ($newSource -eq $source) {
    throw "Patch did not find enableAmplifiers() block. The source file may have changed."
}

$backup = "$Path.bak-bottom-to-top-enable"
Copy-Item $Path $backup -Force
Set-Content -Path $Path -Value $newSource -NoNewline

Write-Host "Patched $Path"
Write-Host "Backup saved to $backup"
Write-Host "Next: cmake --build build-vs2022 --config Release"
