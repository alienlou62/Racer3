using System;
using System.IO;
using System.Text.Json;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class Racer3MotionUiConfig
{
    public string RepoRoot { get; init; } = string.Empty;

    public string LauncherScript { get; init; } = @".\scripts\start-racer3-rmp-and-run.ps1";

    public string PowerShellPath { get; init; } = "powershell";

    public double DefaultVelocity { get; init; } = 0.04;

    public double DefaultShapeSizeMeters { get; init; } = 0.05;

    public CartesianPose DefaultCenter { get; init; } = new(0.50, 0.0, -0.55);

    public bool AutoAcknowledgeConsolePrompt { get; init; } = true;

    public string OpenAiModel { get; init; } = "gpt-4.1-mini";

    public static Racer3MotionUiConfig Load()
    {
        var configPath = Path.Combine(AppContext.BaseDirectory, "Racer3MotionUi.json");
        if (!File.Exists(configPath))
        {
            return new Racer3MotionUiConfig();
        }

        var options = new JsonSerializerOptions
        {
            AllowTrailingCommas = true,
            PropertyNameCaseInsensitive = true,
            ReadCommentHandling = JsonCommentHandling.Skip
        };

        return JsonSerializer.Deserialize<Racer3MotionUiConfig>(File.ReadAllText(configPath), options)
               ?? new Racer3MotionUiConfig();
    }
}
