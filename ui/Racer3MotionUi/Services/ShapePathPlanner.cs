using System;
using System.Collections.Generic;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public sealed class ShapePathPlanner : IShapePathPlanner
{
    public ShapeTracePlan CreatePlan(ShapeKind shape, CartesianPose center, double sizeMeters)
    {
        if (sizeMeters <= 0.0)
        {
            throw new ArgumentOutOfRangeException(nameof(sizeMeters), "Shape size must be greater than zero.");
        }

        var waypoints = shape switch
        {
            ShapeKind.Circle => CreateRadialShape(center, sizeMeters, 16),
            ShapeKind.Square => CreateSquare(center, sizeMeters),
            ShapeKind.Triangle => CreateRadialShape(center, sizeMeters, 3, startAngleRadians: -Math.PI / 2.0),
            ShapeKind.Hexagon => CreateRadialShape(center, sizeMeters, 6),
            _ => throw new ArgumentOutOfRangeException(nameof(shape), shape, "Unsupported shape.")
        };

        return new ShapeTracePlan(shape, center, sizeMeters, waypoints);
    }

    private static IReadOnlyList<CartesianPose> CreateRadialShape(
        CartesianPose center,
        double radius,
        int segments,
        double startAngleRadians = 0.0)
    {
        var points = new List<CartesianPose>(segments + 1);
        for (var index = 0; index < segments; index++)
        {
            var angle = startAngleRadians + (2.0 * Math.PI * index / segments);
            points.Add(new CartesianPose(
                center.X + radius * Math.Cos(angle),
                center.Y + radius * Math.Sin(angle),
                center.Z,
                center.Roll,
                center.Pitch,
                center.Yaw));
        }

        points.Add(points[0]);
        return points;
    }

    private static IReadOnlyList<CartesianPose> CreateSquare(CartesianPose center, double halfSideMeters)
    {
        var points = new List<CartesianPose>
        {
            new(center.X - halfSideMeters, center.Y - halfSideMeters, center.Z, center.Roll, center.Pitch, center.Yaw),
            new(center.X + halfSideMeters, center.Y - halfSideMeters, center.Z, center.Roll, center.Pitch, center.Yaw),
            new(center.X + halfSideMeters, center.Y + halfSideMeters, center.Z, center.Roll, center.Pitch, center.Yaw),
            new(center.X - halfSideMeters, center.Y + halfSideMeters, center.Z, center.Roll, center.Pitch, center.Yaw),
            new(center.X - halfSideMeters, center.Y - halfSideMeters, center.Z, center.Roll, center.Pitch, center.Yaw)
        };

        return points;
    }
}
