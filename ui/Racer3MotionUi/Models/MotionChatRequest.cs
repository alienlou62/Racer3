namespace Racer3MotionUi.Models;

public sealed class MotionChatRequest
{
    public MotionChatRequest(
        string userCommand,
        MotionPlan currentPlan,
        MotionPlan defaultPlan)
    {
        UserCommand = userCommand;
        CurrentPlan = currentPlan;
        DefaultPlan = defaultPlan;
    }

    public string UserCommand { get; }

    public MotionPlan CurrentPlan { get; }

    public MotionPlan DefaultPlan { get; }
}
