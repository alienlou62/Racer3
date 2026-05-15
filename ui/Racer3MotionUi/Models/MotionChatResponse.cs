namespace Racer3MotionUi.Models;

public sealed class MotionChatResponse
{
    public MotionChatResponse(
        MotionChatAction action,
        string assistantMessage,
        MotionPlan? plan = null,
        bool canExecute = false)
    {
        Action = action;
        AssistantMessage = assistantMessage;
        Plan = plan;
        CanExecute = canExecute;
    }

    public MotionChatAction Action { get; }

    public string AssistantMessage { get; }

    public MotionPlan? Plan { get; }

    public bool CanExecute { get; }
}
