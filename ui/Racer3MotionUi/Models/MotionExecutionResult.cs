namespace Racer3MotionUi.Models;

public sealed class MotionExecutionResult
{
    public MotionExecutionResult(int exitCode, int executedCommandCount)
    {
        ExitCode = exitCode;
        ExecutedCommandCount = executedCommandCount;
    }

    public int ExitCode { get; }

    public int ExecutedCommandCount { get; }

    public bool Succeeded => ExitCode == 0;
}
