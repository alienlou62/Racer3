namespace Racer3MotionUi.Models;

public sealed class MotionPlan
{
    public ShapeKind Shape { get; init; } = ShapeKind.Circle;

    public double CenterX { get; init; } = 0.50;

    public double CenterY { get; init; }

    public double CenterZ { get; init; } = -0.55;

    public double SizeMeters { get; init; } = 0.05;

    public double Velocity { get; init; } = 0.04;

    public string CornerMode { get; init; } = "sharp";
}
