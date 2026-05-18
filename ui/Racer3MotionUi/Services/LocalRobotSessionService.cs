using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class LocalRobotSessionService : IRobotSessionService, IDisposable
{
    private readonly Racer3MotionUiConfig _config;
    private readonly object _sync = new();
    private Process? _process;
    private IProgress<ProcessOutputLine>? _output;
    private TaskCompletionSource<bool>? _readySignal;
    private TaskCompletionSource<MotionExecutionResult>? _motionCompletionSignal;

    public LocalRobotSessionService(Racer3MotionUiConfig config)
    {
        _config = config;
    }

    public bool IsRunning
    {
        get
        {
            lock (_sync)
            {
                return _process is { HasExited: false };
            }
        }
    }

    public async Task StartAsync(IProgress<ProcessOutputLine> output, CancellationToken cancellationToken)
    {
        lock (_sync)
        {
            if (_process is { HasExited: false })
            {
                output.Report(new ProcessOutputLine("ui", "Persistent armed session process is already running."));
                return;
            }
        }

        _output = output;

        var repoRoot = ResolveRepoRoot();
        var executablePath = ResolveSessionExecutablePath(repoRoot);

        if (_config.SessionRunRsiconfig)
        {
            await RunRsiconfigAsync(repoRoot, output, cancellationToken).ConfigureAwait(false);
        }
        else
        {
            output.Report(new ProcessOutputLine("ui", "Skipping rsiconfig before session start by config. RMP must already be configured."));
        }

        var readySignal = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        Process process;

        lock (_sync)
        {
            _readySignal = readySignal;
            _process = new Process
            {
                StartInfo = CreateSessionStartInfo(repoRoot, executablePath),
                EnableRaisingEvents = true
            };

            _process.StartInfo.ArgumentList.Add("--session-server");
            _process.Exited += (_, _) => output.Report(new ProcessOutputLine("ui", "Persistent armed session process exited."));
            process = _process;
        }

        output.Report(new ProcessOutputLine("ui", "Starting persistent armed session process. This phase connects RMP and enables amps once; trace/jog commands are still rejected."));

        if (!process.Start())
        {
            throw new InvalidOperationException("Failed to start persistent armed session process.");
        }

        _ = Task.Run(() => PumpOutputAsync(process, output), CancellationToken.None);
        _ = Task.Run(() => PumpErrorAsync(process, output), CancellationToken.None);

        await WaitForReadyAsync(readySignal, TimeSpan.FromSeconds(_config.SessionReadyTimeoutSeconds), cancellationToken).ConfigureAwait(false);
    }

    public Task RequestStatusAsync(CancellationToken cancellationToken)
    {
        return SendCommandAsync("{\"type\":\"status\"}", cancellationToken);
    }

    public Task StopMotionAsync(CancellationToken cancellationToken)
    {
        return SendCommandAsync("{\"type\":\"stop\"}", cancellationToken);
    }

    public async Task<MotionExecutionResult> TraceShapeAsync(
        ShapeTracePlan plan,
        RobotMotionOptions options,
        IProgress<ProcessOutputLine> output,
        CancellationToken cancellationToken)
    {
        if (options.DryRun || !options.ConfirmMotion)
        {
            output.Report(new ProcessOutputLine("ui", "Persistent armed session trace requires live mode with Confirm Motion armed."));
            return new MotionExecutionResult(1, 0);
        }

        if (!IsRunning)
        {
            output.Report(new ProcessOutputLine("ui", "Cannot run a session trace because no persistent armed session is running."));
            return new MotionExecutionResult(1, 0);
        }

        var completion = new TaskCompletionSource<MotionExecutionResult>(TaskCreationOptions.RunContinuationsAsynchronously);
        lock (_sync)
        {
            _motionCompletionSignal = completion;
        }

        var cartesianTrace = string.Join(";", plan.Waypoints.Select(waypoint => waypoint.ToCartesianArgument()));
        var velocityText = options.Velocity.ToString("0.######", CultureInfo.InvariantCulture);
        var returnToZero = _config.SessionTraceReturnToZero ? "true" : "false";
        var command = "{\"type\":\"trace\",\"velocity\":" + velocityText + ",\"returnToZero\":" + returnToZero + ",\"cartesianTrace\":\"" + cartesianTrace + "\"}";

        output.Report(new ProcessOutputLine("ui", $"Session trace requested for {plan.Shape}: {plan.Waypoints.Count} waypoint(s). Amps should remain enabled after completion."));
        await SendCommandAsync(command, cancellationToken).ConfigureAwait(false);

        using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(_config.SessionMotionTimeoutSeconds));
        using var linked = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, timeout.Token);

        try
        {
            using (linked.Token.Register(() => completion.TrySetCanceled(linked.Token)))
            {
                return await completion.Task.ConfigureAwait(false);
            }
        }
        catch (OperationCanceledException) when (timeout.IsCancellationRequested)
        {
            output.Report(new ProcessOutputLine("err", "Timed out waiting for persistent session trace completion."));
            return new MotionExecutionResult(1, 1);
        }
        finally
        {
            lock (_sync)
            {
                if (ReferenceEquals(_motionCompletionSignal, completion))
                {
                    _motionCompletionSignal = null;
                }
            }
        }
    }


    public async Task<MotionExecutionResult> JogAsync(
        double deltaX,
        double deltaY,
        double deltaZ,
        double velocity,
        bool confirmMotion,
        IProgress<ProcessOutputLine> output,
        CancellationToken cancellationToken)
    {
        if (!confirmMotion)
        {
            output.Report(new ProcessOutputLine("ui", "Persistent armed session jog requires Confirm Motion armed."));
            return new MotionExecutionResult(1, 0);
        }

        if (!IsRunning)
        {
            output.Report(new ProcessOutputLine("ui", "Cannot jog because no persistent armed session is running."));
            return new MotionExecutionResult(1, 0);
        }

        var completion = new TaskCompletionSource<MotionExecutionResult>(TaskCreationOptions.RunContinuationsAsynchronously);
        lock (_sync)
        {
            _motionCompletionSignal = completion;
        }

        var command = string.Format(
            CultureInfo.InvariantCulture,
            "{{\"type\":\"cartesian_jog\",\"dx\":{0:0.######},\"dy\":{1:0.######},\"dz\":{2:0.######},\"velocity\":{3:0.######}}}",
            deltaX,
            deltaY,
            deltaZ,
            velocity);

        output.Report(new ProcessOutputLine("ui", string.Format(CultureInfo.InvariantCulture, "Session jog requested: dX={0:0.####} m, dY={1:0.####} m, dZ={2:0.####} m, velocity={3:0.####}.", deltaX, deltaY, deltaZ, velocity)));
        await SendCommandAsync(command, cancellationToken).ConfigureAwait(false);

        using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(_config.SessionMotionTimeoutSeconds));
        using var linked = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, timeout.Token);

        try
        {
            using (linked.Token.Register(() => completion.TrySetCanceled(linked.Token)))
            {
                return await completion.Task.ConfigureAwait(false);
            }
        }
        catch (OperationCanceledException) when (timeout.IsCancellationRequested)
        {
            output.Report(new ProcessOutputLine("err", "Timed out waiting for persistent session jog completion."));
            return new MotionExecutionResult(1, 1);
        }
        finally
        {
            lock (_sync)
            {
                if (ReferenceEquals(_motionCompletionSignal, completion))
                {
                    _motionCompletionSignal = null;
                }
            }
        }
    }

    public async Task ShutdownAsync(CancellationToken cancellationToken)
    {
        Process? process;
        lock (_sync)
        {
            process = _process;
        }

        if (process is null || process.HasExited)
        {
            _output?.Report(new ProcessOutputLine("ui", "No persistent armed session process is running."));
            return;
        }

        await SendCommandAsync("{\"type\":\"shutdown\"}", cancellationToken).ConfigureAwait(false);

        using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(20));
        using var linked = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, timeout.Token);

        try
        {
            await process.WaitForExitAsync(linked.Token).ConfigureAwait(false);
        }
        catch (OperationCanceledException) when (timeout.IsCancellationRequested)
        {
            if (!process.HasExited)
            {
                process.Kill(entireProcessTree: true);
                _output?.Report(new ProcessOutputLine("ui", "Persistent armed session process did not exit after shutdown request and was killed."));
            }
        }
        finally
        {
            lock (_sync)
            {
                _process?.Dispose();
                _process = null;
            }
        }
    }

    public void Dispose()
    {
        lock (_sync)
        {
            if (_process is { HasExited: false })
            {
                _process.Kill(entireProcessTree: true);
            }

            _process?.Dispose();
            _process = null;
        }
    }

    private async Task RunRsiconfigAsync(string repoRoot, IProgress<ProcessOutputLine> output, CancellationToken cancellationToken)
    {
        var settingsPath = Path.Combine(repoRoot, "config", "racer3-settings.xml");
        if (!File.Exists(settingsPath))
        {
            throw new FileNotFoundException("Racer3 settings file was not found for session rsiconfig.", settingsPath);
        }

        var startInfo = new ProcessStartInfo
        {
            FileName = "rsiconfig",
            WorkingDirectory = Path.GetDirectoryName(settingsPath) ?? repoRoot,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true
        };
        startInfo.ArgumentList.Add("racer3-settings.xml");
        startInfo.ArgumentList.Add("--verbose");
        ApplyRuntimePath(startInfo, repoRoot, Path.Combine(repoRoot, "build-vs2022", "Release"));

        output.Report(new ProcessOutputLine("ui", "Starting/configuring RMP once for persistent armed session with rsiconfig."));

        using var process = new Process { StartInfo = startInfo };
        process.Start();

        var stdoutTask = PumpRsiconfigStreamAsync(process.StandardOutput, "rsiconfig", output);
        var stderrTask = PumpRsiconfigStreamAsync(process.StandardError, "rsiconfig-err", output);

        await process.WaitForExitAsync(cancellationToken).ConfigureAwait(false);
        await Task.WhenAll(stdoutTask, stderrTask).ConfigureAwait(false);

        if (process.ExitCode != 0)
        {
            throw new InvalidOperationException($"rsiconfig failed with exit code {process.ExitCode}.");
        }

        output.Report(new ProcessOutputLine("ui", "rsiconfig completed for persistent armed session."));
    }

    private static async Task PumpRsiconfigStreamAsync(StreamReader reader, string stream, IProgress<ProcessOutputLine> output)
    {
        while (true)
        {
            var line = await reader.ReadLineAsync().ConfigureAwait(false);
            if (line is null)
            {
                break;
            }

            if (!string.IsNullOrWhiteSpace(line))
            {
                output.Report(new ProcessOutputLine(stream, line));
            }
        }
    }

    private async Task SendCommandAsync(string command, CancellationToken cancellationToken)
    {
        Process? process;
        lock (_sync)
        {
            process = _process;
        }

        if (process is null || process.HasExited)
        {
            _output?.Report(new ProcessOutputLine("ui", "Cannot send session command because no persistent armed session process is running."));
            return;
        }

        cancellationToken.ThrowIfCancellationRequested();
        await process.StandardInput.WriteLineAsync(command).ConfigureAwait(false);
        await process.StandardInput.FlushAsync().ConfigureAwait(false);
        _output?.Report(new ProcessOutputLine("session-cmd", command));
    }

    private async Task PumpOutputAsync(Process process, IProgress<ProcessOutputLine> output)
    {
        try
        {
            while (!process.HasExited)
            {
                var line = await process.StandardOutput.ReadLineAsync().ConfigureAwait(false);
                if (line is null)
                {
                    break;
                }

                output.Report(new ProcessOutputLine("session", line));

                if (line.Contains("\"type\":\"session_ready\"", StringComparison.OrdinalIgnoreCase))
                {
                    _readySignal?.TrySetResult(true);
                }

                if (line.Contains("\"type\":\"session_error\"", StringComparison.OrdinalIgnoreCase))
                {
                    _readySignal?.TrySetException(new InvalidOperationException(line));
                }

                if (line.Contains("\"type\":\"session_trace_complete\"", StringComparison.OrdinalIgnoreCase) ||
                    line.Contains("\"type\":\"session_jog_complete\"", StringComparison.OrdinalIgnoreCase))
                {
                    _motionCompletionSignal?.TrySetResult(new MotionExecutionResult(0, 1));
                }

                if (line.Contains("\"type\":\"session_trace_error\"", StringComparison.OrdinalIgnoreCase) ||
                    line.Contains("\"type\":\"session_jog_error\"", StringComparison.OrdinalIgnoreCase) ||
                    line.Contains("\"type\":\"session_reject\"", StringComparison.OrdinalIgnoreCase))
                {
                    _motionCompletionSignal?.TrySetResult(new MotionExecutionResult(1, 1));
                }
            }
        }
        catch (Exception exception)
        {
            output.Report(new ProcessOutputLine("err", $"Session stdout reader error: {exception.Message}"));
        }
    }

    private static async Task PumpErrorAsync(Process process, IProgress<ProcessOutputLine> output)
    {
        try
        {
            while (!process.HasExited)
            {
                var line = await process.StandardError.ReadLineAsync().ConfigureAwait(false);
                if (line is null)
                {
                    break;
                }

                output.Report(new ProcessOutputLine("session-err", line));
            }
        }
        catch (Exception exception)
        {
            output.Report(new ProcessOutputLine("err", $"Session stderr reader error: {exception.Message}"));
        }
    }

    private static async Task WaitForReadyAsync(TaskCompletionSource<bool> readySignal, TimeSpan timeout, CancellationToken cancellationToken)
    {
        var completed = await Task.WhenAny(
                readySignal.Task,
                Task.Delay(timeout, cancellationToken))
            .ConfigureAwait(false);

        if (completed != readySignal.Task)
        {
            throw new TimeoutException($"Persistent armed session process did not report ready within {timeout.TotalSeconds:0} seconds.");
        }

        await readySignal.Task.ConfigureAwait(false);
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

    private string ResolveSessionExecutablePath(string repoRoot)
    {
        var configuredPath = _config.SessionExecutablePath;
        var fullPath = Path.IsPathRooted(configuredPath)
            ? configuredPath
            : Path.GetFullPath(Path.Combine(repoRoot, configuredPath));

        if (!File.Exists(fullPath))
        {
            throw new FileNotFoundException("Persistent armed session executable was not found.", fullPath);
        }

        return fullPath;
    }

    private ProcessStartInfo CreateSessionStartInfo(string repoRoot, string executablePath)
    {
        var startInfo = new ProcessStartInfo
        {
            FileName = executablePath,
            WorkingDirectory = repoRoot,
            UseShellExecute = false,
            RedirectStandardInput = true,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true
        };

        ApplyRuntimePath(startInfo, repoRoot, Path.GetDirectoryName(executablePath) ?? repoRoot);
        return startInfo;
    }

    private void ApplyRuntimePath(ProcessStartInfo startInfo, string repoRoot, string executableDirectory)
    {
        var existingPath = Environment.GetEnvironmentVariable("PATH") ?? string.Empty;
        startInfo.Environment["PATH"] = $"{_config.RsiRuntimePath};{executableDirectory};{existingPath}";
    }
}
