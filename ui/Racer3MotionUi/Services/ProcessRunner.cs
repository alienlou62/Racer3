using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class ProcessRunner
{
    public async Task<int> RunAsync(
        MotionCommand command,
        bool autoAcknowledgeConsolePrompt,
        IProgress<ProcessOutputLine> output,
        CancellationToken cancellationToken)
    {
        using var process = new Process
        {
            StartInfo = new ProcessStartInfo
            {
                FileName = command.Executable,
                WorkingDirectory = command.WorkingDirectory,
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                RedirectStandardInput = true,
                CreateNoWindow = true
            },
            EnableRaisingEvents = true
        };

        foreach (var argument in command.Arguments)
        {
            process.StartInfo.ArgumentList.Add(argument);
        }

        process.OutputDataReceived += (_, args) =>
        {
            if (!string.IsNullOrEmpty(args.Data))
            {
                output.Report(new ProcessOutputLine("out", args.Data));
            }
        };

        process.ErrorDataReceived += (_, args) =>
        {
            if (!string.IsNullOrEmpty(args.Data))
            {
                output.Report(new ProcessOutputLine("err", args.Data));
            }
        };

        if (!process.Start())
        {
            throw new InvalidOperationException("Failed to start the robot motion process.");
        }

        process.BeginOutputReadLine();
        process.BeginErrorReadLine();

        if (autoAcknowledgeConsolePrompt)
        {
            await process.StandardInput.WriteLineAsync();
            process.StandardInput.Close();
            output.Report(new ProcessOutputLine("ui", "Sent operator acknowledgement newline to the console process."));
        }

        try
        {
            await process.WaitForExitAsync(cancellationToken);
        }
        catch (OperationCanceledException)
        {
            if (!process.HasExited)
            {
                process.Kill(entireProcessTree: true);
                output.Report(new ProcessOutputLine("ui", "Stopped the robot motion process."));
            }

            throw;
        }

        return process.ExitCode;
    }
}
