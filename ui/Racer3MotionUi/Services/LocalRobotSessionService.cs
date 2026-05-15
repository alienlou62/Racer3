using System;
using System.Diagnostics;
using System.IO;
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
                output.Report(new ProcessOutputLine("ui", "Persistent session process is already running."));
                return;
            }

            _output = output;
            _readySignal = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);

            var repoRoot = ResolveRepoRoot();
            var executablePath = ResolveSessionExecutablePath(repoRoot);

            _process = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = executablePath,
                    WorkingDirectory = repoRoot,
                    UseShellExecute = false,
                    RedirectStandardInput = true,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                },
                EnableRaisingEvents = true
            };

            _process.StartInfo.ArgumentList.Add("--session-server");
            _process.Exited += (_, _) => output.Report(new ProcessOutputLine("ui", "Persistent session process exited."));
        }

        Process process;
        TaskCompletionSource<bool> readySignal;
        lock (_sync)
        {
            process = _process ?? throw new InvalidOperationException("Session process was not initialized.");
            readySignal = _readySignal ?? throw new InvalidOperationException("Session ready signal was not initialized.");
        }

        output.Report(new ProcessOutputLine("ui", "Starting persistent session process skeleton. This phase does not connect RMP or enable amps."));

        if (!process.Start())
        {
            throw new InvalidOperationException("Failed to start persistent session process.");
        }

        _ = Task.Run(() => PumpOutputAsync(process, output), CancellationToken.None);
        _ = Task.Run(() => PumpErrorAsync(process, output), CancellationToken.None);

        await WaitForReadyAsync(readySignal, cancellationToken).ConfigureAwait(false);
    }

    public Task RequestStatusAsync(CancellationToken cancellationToken)
    {
        return SendCommandAsync("{\"type\":\"status\"}", cancellationToken);
    }

    public Task StopMotionAsync(CancellationToken cancellationToken)
    {
        return SendCommandAsync("{\"type\":\"stop\"}", cancellationToken);
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
            _output?.Report(new ProcessOutputLine("ui", "No persistent session process is running."));
            return;
        }

        await SendCommandAsync("{\"type\":\"shutdown\"}", cancellationToken).ConfigureAwait(false);

        using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(5));
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
                _output?.Report(new ProcessOutputLine("ui", "Persistent session process did not exit after shutdown request and was killed."));
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

    private async Task SendCommandAsync(string command, CancellationToken cancellationToken)
    {
        Process? process;
        lock (_sync)
        {
            process = _process;
        }

        if (process is null || process.HasExited)
        {
            _output?.Report(new ProcessOutputLine("ui", "Cannot send session command because no persistent session process is running."));
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

    private static async Task WaitForReadyAsync(TaskCompletionSource<bool> readySignal, CancellationToken cancellationToken)
    {
        var completed = await Task.WhenAny(
                readySignal.Task,
                Task.Delay(TimeSpan.FromSeconds(5), cancellationToken))
            .ConfigureAwait(false);

        if (completed != readySignal.Task)
        {
            throw new TimeoutException("Persistent session process did not report ready within 5 seconds.");
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
            throw new FileNotFoundException("Persistent session executable was not found.", fullPath);
        }

        return fullPath;
    }
}
