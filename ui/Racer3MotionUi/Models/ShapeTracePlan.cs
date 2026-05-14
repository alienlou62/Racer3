using System.Collections.Generic;
using System.Linq;

namespace Racer3MotionUi.Models;

public sealed class ShapeTracePlan
{
    public ShapeTracePlan(
        ShapeKind shape,
        CartesianPose center,
        double sizeMeters,
        IReadOnlyList<CartesianPose> waypoints)
    {
        Shape = shape;
        Center = center;
        SizeMeters = sizeMeters;
        Waypoints = waypoints;
    }

    public ShapeKind Shape { get; }

    public CartesianPose Center { get; }

    public double SizeMeters { get; }

    public IReadOnlyList<CartesianPose> Waypoints { get; }

    public CartesianPose RepresentativePose => Waypoints.FirstOrDefault();
}
