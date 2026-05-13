param(
    [string]$CppPath = ".\src\Racer3BasicMotion.cpp",
    [string]$HeaderPath = ".\src\Racer3BasicMotion.h"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $CppPath)) {
    throw "Could not find $CppPath. Run this from the repo root or pass -CppPath."
}
if (-not (Test-Path $HeaderPath)) {
    throw "Could not find $HeaderPath. Run this from the repo root or pass -HeaderPath."
}

$cpp = Get-Content -Raw -Path $CppPath
$header = Get-Content -Raw -Path $HeaderPath

# 1) Add private method declaration to the header.
if ($header -notmatch 'clearFaultsAfterCompletedMotion') {
    $headerNew = $header.Replace(
        '    void clearFaults();',
        "    void clearFaults();`r`n    void clearFaultsAfterCompletedMotion(const char* context) noexcept;"
    )

    if ($headerNew -eq $header) {
        throw "Could not insert clearFaultsAfterCompletedMotion declaration after clearFaults()."
    }

    Copy-Item $HeaderPath "$HeaderPath.bak-post-motion-clear" -Force
    Set-Content -Path $HeaderPath -Value $headerNew -NoNewline
    Write-Host "Patched $HeaderPath"
} else {
    Write-Host "$HeaderPath already has clearFaultsAfterCompletedMotion declaration; skipping header patch."
}

# Re-read cpp in case paths are shared with generated source.
$cpp = Get-Content -Raw -Path $CppPath

# 2) Add implementation after clearFaults().
if ($cpp -notmatch 'Racer3BasicMotion::clearFaultsAfterCompletedMotion') {
    $pattern = '(?s)(void Racer3BasicMotion::clearFaults\(\)\s*\{.*?\n\}\s*)\n(void Racer3BasicMotion::enableAmplifiers\(\))'

    $helper = @'

void Racer3BasicMotion::clearFaultsAfterCompletedMotion(const char* context) noexcept
{
    const char* label = context ? context : "completed motion";
    std::cout << "Clearing faults after "
              << label
              << " before final program exit...\n";

    bool warning = false;

    if (multiAxis_)
    {
        try
        {
            multiAxis_->ClearFaults();
            std::cout << "  MultiAxis 6 post-motion fault clear sent.\n";
        }
        catch (const RR::RsiError& error)
        {
            warning = true;
            std::cout << "  MultiAxis 6 post-motion fault clear threw RapidCode error: "
                      << error.text
                      << " ("
                      << error.functionName
                      << ")\n";
        }
        catch (const std::exception& error)
        {
            warning = true;
            std::cout << "  MultiAxis 6 post-motion fault clear threw exception: "
                      << error.what()
                      << "\n";
        }
        catch (...)
        {
            warning = true;
            std::cout << "  MultiAxis 6 post-motion fault clear threw unknown exception.\n";
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
            axes_[index]->ClearFaults();
            std::cout << "  Axis "
                      << (index + 1)
                      << " post-motion fault clear sent.\n";
        }
        catch (const RR::RsiError& error)
        {
            warning = true;
            std::cout << "  Axis "
                      << (index + 1)
                      << " post-motion fault clear threw RapidCode error: "
                      << error.text
                      << " ("
                      << error.functionName
                      << ")\n";
        }
        catch (const std::exception& error)
        {
            warning = true;
            std::cout << "  Axis "
                      << (index + 1)
                      << " post-motion fault clear threw exception: "
                      << error.what()
                      << "\n";
        }
        catch (...)
        {
            warning = true;
            std::cout << "  Axis "
                      << (index + 1)
                      << " post-motion fault clear threw unknown exception.\n";
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(FaultClearSettleMs));

    std::cout << "Post-motion fault clear complete"
              << (warning ? " with warnings.\n" : ".\n");
}

'@

    $cppNew = [regex]::Replace($cpp, $pattern, ('$1' + $helper + '$2'), 1)

    if ($cppNew -eq $cpp) {
        throw "Could not insert clearFaultsAfterCompletedMotion implementation after clearFaults()."
    }

    Copy-Item $CppPath "$CppPath.bak-post-motion-clear" -Force
    Set-Content -Path $CppPath -Value $cppNew -NoNewline
    Write-Host "Inserted clearFaultsAfterCompletedMotion implementation in $CppPath"
} else {
    Write-Host "$CppPath already has clearFaultsAfterCompletedMotion implementation; skipping implementation patch."
}

# 3) Add successful-motion cleanup calls after disableAmplifiers().
$cpp = Get-Content -Raw -Path $CppPath
$changed = $false

$oldSmooth = @'
        disableAmplifiers();

        std::cout << "Guarded smooth endpoint Cartesian-vector motion complete.\n";
'@
$newSmooth = @'
        disableAmplifiers();
        clearFaultsAfterCompletedMotion("smooth endpoint Cartesian-vector motion");

        std::cout << "Guarded smooth endpoint Cartesian-vector motion complete.\n";
'@

if ($cpp.Contains($oldSmooth) -and -not $cpp.Contains('clearFaultsAfterCompletedMotion("smooth endpoint Cartesian-vector motion")')) {
    $cpp = $cpp.Replace($oldSmooth, $newSmooth)
    $changed = $true
    Write-Host "Added post-motion fault clear to smooth endpoint Cartesian-vector success path."
}

$oldSegmented = @'
    disableAmplifiers();

    std::cout << "Guarded segmented Cartesian-vector motion complete.\n";
'@
$newSegmented = @'
    disableAmplifiers();
    clearFaultsAfterCompletedMotion("segmented Cartesian-vector motion");

    std::cout << "Guarded segmented Cartesian-vector motion complete.\n";
'@

if ($cpp.Contains($oldSegmented) -and -not $cpp.Contains('clearFaultsAfterCompletedMotion("segmented Cartesian-vector motion")')) {
    $cpp = $cpp.Replace($oldSegmented, $newSegmented)
    $changed = $true
    Write-Host "Added post-motion fault clear to segmented Cartesian-vector success path."
}

if ($changed) {
    Set-Content -Path $CppPath -Value $cpp -NoNewline
    Write-Host "Patched successful motion shutdown cleanup in $CppPath"
} else {
    Write-Host "No successful motion shutdown cleanup changes were needed."
}

Write-Host "Next: cmake --build build-vs2022 --config Release"
