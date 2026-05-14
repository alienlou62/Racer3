using System.Collections.Generic;

namespace Racer3MotionUi.Models;

public sealed class MotionCommand
{
    public MotionCommand(
        string executable,
        IReadOnlyList<string> arguments,
        string workingDirectory,
        string displayText)
    {
        Executable = executable;
        Arguments = arguments;
        WorkingDirectory = workingDirectory;
        DisplayText = displayText;
    }

    public string Executable { get; }

    public IReadOnlyList<string> Arguments { get; }

    public string WorkingDirectory { get; }

    public string DisplayText { get; }
}
