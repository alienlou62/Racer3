using System;
using System.Globalization;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class RuleBasedMotionChatService : IMotionChatService
{
    private const double VelocityStep = 0.005;
    private const double SizeScaleUp = 1.20;
    private const double SizeScaleDown = 0.80;
    private const double CenterStepMeters = 0.01;

    public string StatusText => "Local parser fallback";

    public bool IsAvailable => true;

    public Task<MotionChatResponse> InterpretAsync(
        MotionChatRequest request,
        CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();

        var command = Normalize(request.UserCommand);
        if (string.IsNullOrWhiteSpace(command))
        {
            return Task.FromResult(new MotionChatResponse(
                MotionChatAction.Explain,
                "Type a command such as draw a bigger square higher up."));
        }

        if (HasExecutionIntent(command))
        {
            return Task.FromResult(new MotionChatResponse(
                MotionChatAction.Explain,
                "I prepared the plan. Validate first, then arm Confirm Motion and press Run Selected Shape."));
        }

        if (command.Contains("reset", StringComparison.Ordinal))
        {
            return Task.FromResult(new MotionChatResponse(
                MotionChatAction.ResetPlan,
                "Reset to safe defaults. Dry Run / Validate Only is enabled and live motion is disarmed.",
                request.DefaultPlan));
        }

        if (MentionsUnsupportedDrawShape(command))
        {
            return Task.FromResult(new MotionChatResponse(
                MotionChatAction.Reject,
                "I can plan circle, square, triangle, or hexagon previews only."));
        }

        if (command.Contains("outside", StringComparison.Ordinal) ||
            command.Contains("unsafe", StringComparison.Ordinal) ||
            command.Contains("collision", StringComparison.Ordinal))
        {
            return Task.FromResult(new MotionChatResponse(
                MotionChatAction.Reject,
                "I cannot plan motion described as outside the safe robot area."));
        }

        var plan = request.CurrentPlan;
        var changed = false;

        if (TryFindShape(command, out var shape))
        {
            plan = Copy(plan, shape: shape);
            changed = true;
        }

        if (TryFindNumber(command, "x", out var x))
        {
            plan = Copy(plan, centerX: x);
            changed = true;
        }

        if (TryFindNumber(command, "y", out var y))
        {
            plan = Copy(plan, centerY: y);
            changed = true;
        }

        if (TryFindNumber(command, "z", out var z))
        {
            plan = Copy(plan, centerZ: z);
            changed = true;
        }

        if (TryFindNumber(command, "size", out var size) ||
            TryFindNumber(command, "radius", out size))
        {
            plan = Copy(plan, sizeMeters: size);
            changed = true;
        }

        if (TryFindNumber(command, "velocity", out var velocity) ||
            TryFindNumber(command, "speed", out velocity))
        {
            plan = Copy(plan, velocity: velocity);
            changed = true;
        }

        if (ContainsAny(command, "bigger", "larger"))
        {
            plan = Copy(plan, sizeMeters: plan.SizeMeters * SizeScaleUp);
            changed = true;
        }
        else if (ContainsAny(command, "smaller", "small"))
        {
            plan = Copy(plan, sizeMeters: plan.SizeMeters * SizeScaleDown);
            changed = true;
        }
        else if (ContainsAny(command, "huge", "giant"))
        {
            plan = Copy(plan, sizeMeters: plan.SizeMeters * 3.0);
            changed = true;
        }

        if (ContainsAny(command, "higher", "up"))
        {
            plan = Copy(plan, centerZ: plan.CenterZ + CenterStepMeters);
            changed = true;
        }

        if (ContainsAny(command, "lower", "down"))
        {
            plan = Copy(plan, centerZ: plan.CenterZ - CenterStepMeters);
            changed = true;
        }

        if (command.Contains("left", StringComparison.Ordinal))
        {
            plan = Copy(plan, centerY: plan.CenterY - CenterStepMeters);
            changed = true;
        }

        if (command.Contains("right", StringComparison.Ordinal))
        {
            plan = Copy(plan, centerY: plan.CenterY + CenterStepMeters);
            changed = true;
        }

        if (ContainsAny(command, "faster", "speed up"))
        {
            plan = Copy(plan, velocity: plan.Velocity + VelocityStep);
            changed = true;
        }

        if (ContainsAny(command, "slower", "slow down"))
        {
            plan = Copy(plan, velocity: plan.Velocity - VelocityStep);
            changed = true;
        }

        if (!changed)
        {
            return Task.FromResult(new MotionChatResponse(
                MotionChatAction.Explain,
                "I can update shape, center X/Y/Z, size/radius, and velocity for the preview."));
        }

        return Task.FromResult(new MotionChatResponse(
            MotionChatAction.UpdatePlan,
            DescribePlan(plan),
            plan));
    }

    public static bool HasExecutionIntent(string command)
    {
        return ContainsAny(
            Normalize(command),
            "run it",
            "execute",
            "start",
            "move now",
            "confirm motion",
            "begin motion",
            "go now",
            "send it");
    }

    private static string Normalize(string command)
    {
        var cleaned = command.Trim().Trim('.', '!', '?', ',').ToLowerInvariant();
        return string.Join(' ', cleaned.Split(Array.Empty<char>(), StringSplitOptions.RemoveEmptyEntries));
    }

    private static bool TryFindShape(string command, out ShapeKind shape)
    {
        if (command.Contains("circle", StringComparison.Ordinal))
        {
            shape = ShapeKind.Circle;
            return true;
        }

        if (command.Contains("square", StringComparison.Ordinal))
        {
            shape = ShapeKind.Square;
            return true;
        }

        if (command.Contains("triangle", StringComparison.Ordinal))
        {
            shape = ShapeKind.Triangle;
            return true;
        }

        if (command.Contains("hexagon", StringComparison.Ordinal))
        {
            shape = ShapeKind.Hexagon;
            return true;
        }

        shape = ShapeKind.Circle;
        return false;
    }

    private static bool MentionsUnsupportedDrawShape(string command)
    {
        if (!ContainsAny(command, "draw", "make", "plan"))
        {
            return false;
        }

        return ContainsAny(command, "star", "pentagon", "octagon", "line", "spiral", "ellipse");
    }

    private static bool TryFindNumber(string command, string label, out double value)
    {
        var match = Regex.Match(
            command,
            $@"(?:^|\s){Regex.Escape(label)}\s*[:=]?\s*(-?\d+(?:\.\d+)?)",
            RegexOptions.CultureInvariant);

        if (match.Success &&
            double.TryParse(match.Groups[1].Value, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
        {
            return true;
        }

        value = 0.0;
        return false;
    }

    private static bool ContainsAny(string value, params string[] terms)
    {
        foreach (var term in terms)
        {
            if (value.Contains(term, StringComparison.Ordinal))
            {
                return true;
            }
        }

        return false;
    }

    private static MotionPlan Copy(
        MotionPlan source,
        ShapeKind? shape = null,
        double? centerX = null,
        double? centerY = null,
        double? centerZ = null,
        double? sizeMeters = null,
        double? velocity = null)
    {
        return new MotionPlan
        {
            Shape = shape ?? source.Shape,
            CenterX = centerX ?? source.CenterX,
            CenterY = centerY ?? source.CenterY,
            CenterZ = centerZ ?? source.CenterZ,
            SizeMeters = sizeMeters ?? source.SizeMeters,
            Velocity = velocity ?? source.Velocity,
            CornerMode = source.CornerMode
        };
    }

    private static string DescribePlan(MotionPlan plan)
    {
        return FormattableString.Invariant(
            $"I planned a {plan.Shape.ToString().ToLowerInvariant()} preview at X={plan.CenterX:0.00}, Y={plan.CenterY:0.00}, Z={plan.CenterZ:0.00}, size={plan.SizeMeters:0.000} m, velocity={plan.Velocity:0.000} m/s. Please validate before live motion.");
    }
}
