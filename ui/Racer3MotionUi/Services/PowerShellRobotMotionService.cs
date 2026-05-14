using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class PowerShellRobotMotionService : IRobotMotionService
{
    private readonly Racer3MotionUiConfig _config;
    private readonly ProcessRunner _processRunner;

    public PowerShellRobotMotionService(Racer3MotionUiConfig config, ProcessRunner processRunner)
    {
        _config = config;
        _processRunner = processRunner;
    }

    public IReadOnlyList<MotionCommand> BuildTraceCommands(ShapeTracePlan plan, RobotMotionOptions options)
    {
        var repoRoot = ResolveRepoRoot();
        var scriptPath = NormalizeScriptPath(_config.LauncherScript);
        var velocityText = options.Velocity.ToString("0.######", CultureInfo.InvariantCulture);
        var commandOptions = CreateEffectiveCommandOptions(options);

        return new[]
        {
            BuildTraceCommand(repoRoot, scriptPath, velocityText, plan, commandOptions)
        };
    }

    public async Task<MotionExecutionResult> TraceShapeAsync(
        ShapeTracePlan plan,
        RobotMotionOptions options,
        IProgress<ProcessOutputLine> output,
        CancellationToken cancellationToken)
    {
        var commands = BuildTraceCommands(plan, options);
        if (commands.Count == 0)
        {
            throw new InvalidOperationException("No robot commands were generated for the selected shape.");
        }

        if (options.DryRun)
        {
            output.Report(new ProcessOutputLine(
                "ui",
                $"Trace validation run: {plan.Shape} will connect/read RMP and validate {plan.Waypoints.Count} waypoint(s) through cartesian-trace without -ConfirmMotion. No amps or motion should be commanded by the backend."));
        }
        else
        {
            output.Report(new ProcessOutputLine(
                "ui",
                $"Live guarded trace: {plan.Shape} will stream {plan.Waypoints.Count} requested waypoint(s) only after backend validation gates pass."));
        }

        var executedCount = 0;
        for (var index = 0; index < commands.Count; index++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            output.Report(new ProcessOutputLine("ui", $"Starting trace command {index + 1} of {commands.Count}."));

            var exitCode = await _processRunner.RunAsync(
                commands[index],
                _config.AutoAcknowledgeConsolePrompt,
                output,
                cancellationToken);

            executedCount++;
            output.Report(new ProcessOutputLine("ui", $"Trace command {index + 1} finished with exit code {exitCode}."));

            if (exitCode != 0)
            {
                output.Report(new ProcessOutputLine("err", "Stopping after the failed trace command."));
                return new MotionExecutionResult(exitCode, executedCount);
            }
        }

        return new MotionExecutionResult(0, executedCount);
    }

    private string ResolveRepoRoot()
    {
        if (!string.IsNullOrWhiteSpace(_config.RepoRoot))
        {
            return Path.GetFullPath(_config.RepoRoot);
        }

        var directory = new DirectoryInfo(AppContext.BaseDirectory);
        while (directory != null)
        {
            if (File.Exists(Path.Combine(directory.FullName, "CMakeLists.txt")) &&
                Directory.Exists(Path.Combine(directory.FullName, "scripts")))
            {
                return directory.FullName;
            }

            directory = directory.Parent;
        }

        return Directory.GetCurrentDirectory();
    }

    private static string NormalizeScriptPath(string configuredPath)
    {
        if (Path.IsPathRooted(configuredPath))
        {
            return configuredPath;
        }

        return configuredPath.StartsWith(@".\", StringComparison.Ordinal)
            ? configuredPath
            : @".\" + configuredPath.TrimStart('\\', '/');
    }

    private MotionCommand BuildTraceCommand(
        string repoRoot,
        string scriptPath,
        string velocityText,
        ShapeTracePlan plan,
        RobotMotionOptions options)
    {
        var cartesianTrace = string.Join(";", plan.Waypoints.Select(waypoint => waypoint.ToCartesianArgument()));
        var arguments = new List<string>
        {
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            scriptPath,
            "-Mode",
            "cartesian-trace",
            "-PositionOnly",
            "-EndpointOnly",
            "-CompactMotion",
            "-CartesianTrace",
            cartesianTrace,
            "-Velocity",
            velocityText
        };

        if (options.ConfirmMotion)
        {
            arguments.Add("-ConfirmMotion");
        }

        return new MotionCommand(
            _config.PowerShellPath,
            arguments,
            repoRoot,
            BuildTraceDisplayText(_config.PowerShellPath, scriptPath, cartesianTrace, velocityText, options));
    }

    private static RobotMotionOptions CreateEffectiveCommandOptions(RobotMotionOptions options)
    {
        return new RobotMotionOptions
        {
            Velocity = options.Velocity,
            DryRun = false,
            ConfirmMotion = options.ConfirmMotion && !options.DryRun
        };
    }

    private static string BuildTraceDisplayText(
        string powerShellPath,
        string scriptPath,
        string cartesianTrace,
        string velocityText,
        RobotMotionOptions options)
    {
        var lines = new List<string>
        {
            powerShellPath + " -ExecutionPolicy Bypass -File " + scriptPath + " `",
            "  -Mode cartesian-trace `",
            "  -PositionOnly `",
            "  -EndpointOnly `",
            "  -CompactMotion `",
            "  -CartesianTrace \"" + cartesianTrace + "\" `",
            "  -Velocity " + velocityText
        };

        if (options.ConfirmMotion)
        {
            lines[^1] += " `";
            lines.Add("  -ConfirmMotion");
        }

        return string.Join(Environment.NewLine, lines);
    }
}
