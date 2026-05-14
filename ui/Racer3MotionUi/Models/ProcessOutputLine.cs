namespace Racer3MotionUi.Models;

public sealed class ProcessOutputLine
{
    public ProcessOutputLine(string stream, string text)
    {
        Stream = stream;
        Text = text;
    }

    public string Stream { get; }

    public string Text { get; }
}
