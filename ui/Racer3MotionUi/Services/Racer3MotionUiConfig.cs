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

    public string SessionExecutablePath { get; init; } = @".\build-vs2022\Release\racer3-basic-motion.exe";

    public string RsiRuntimePath { get; init; } = @"C:\RSI\11.0.0";

    public bool SessionRunRsiconfig { get; init; } = true;

    public int SessionReadyTimeoutSeconds { get; init; } = 90;

    public double DefaultVelocity { get; init; } = 0.04;

    public double DefaultShapeSizeMeters { get; init; } = 0.05;

    public CartesianPose DefaultCenter { get; init; } = new(0.50, 0.0, -0.55);

    public bool AutoAcknowledgeConsolePrompt { get; init; } = true;

    public string ChatProvider { get; init; } = "LocalRules";

    public string OpenAiModel { get; init; } = "gpt-4.1-mini";

    public string OllamaBaseUrl { get; init; } = "http://localhost:11434";

    public string OllamaModel { get; init; } = "qwen2.5:7b";

    public string GeminiModel { get; init; } = "gemini-2.5-flash";

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
