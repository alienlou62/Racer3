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
            ShapeKind.Circle => CreateRadialShapeInYzPlane(center, sizeMeters, 16),
            ShapeKind.Square => CreateSquareInYzPlane(center, sizeMeters),
            ShapeKind.Triangle => CreateCornerHeldClosedShapeInYzPlane(
                CreateRadialShapeInYzPlane(center, sizeMeters, 3, startAngleRadians: -Math.PI / 2.0)),
            ShapeKind.Hexagon => CreateCornerHeldClosedShapeInYzPlane(
                CreateRadialShapeInYzPlane(center, sizeMeters, 6)),
            _ => throw new ArgumentOutOfRangeException(nameof(shape), shape, "Unsupported shape.")
        };

        return new ShapeTracePlan(shape, center, sizeMeters, waypoints);
    }

    private static IReadOnlyList<CartesianPose> CreateRadialShapeInYzPlane(
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
                center.X,
                center.Y + radius * Math.Cos(angle),
                center.Z + radius * Math.Sin(angle),
                center.Roll,
                center.Pitch,
                center.Yaw));
        }

        points.Add(points[0]);
        return points;
    }

    private static IReadOnlyList<CartesianPose> CreateSquareInYzPlane(CartesianPose center, double halfSideMeters)
    {
        var points = new List<CartesianPose>
        {
            new(center.X, center.Y - halfSideMeters, center.Z - halfSideMeters, center.Roll, center.Pitch, center.Yaw),
            new(center.X, center.Y + halfSideMeters, center.Z - halfSideMeters, center.Roll, center.Pitch, center.Yaw),
            new(center.X, center.Y + halfSideMeters, center.Z + halfSideMeters, center.Roll, center.Pitch, center.Yaw),
            new(center.X, center.Y - halfSideMeters, center.Z + halfSideMeters, center.Roll, center.Pitch, center.Yaw),
            new(center.X, center.Y - halfSideMeters, center.Z - halfSideMeters, center.Roll, center.Pitch, center.Yaw)
        };

        return CreateCornerHeldClosedShapeInYzPlane(points);
    }

    private static IReadOnlyList<CartesianPose> CreateCornerHeldClosedShapeInYzPlane(IReadOnlyList<CartesianPose> closedPoints)
    {
        if (closedPoints.Count < 2)
        {
            return closedPoints;
        }

        // Duplicate polygon vertices so the generated PVT trace explicitly reaches
        // zero velocity at each corner before starting the next edge. This keeps
        // polygon corners visually sharper without changing the robot backend or
        // the smooth circle path.
        var cornerHeldPoints = new List<CartesianPose>(closedPoints.Count * 2);

        foreach (var point in closedPoints)
        {
            cornerHeldPoints.Add(point);
            cornerHeldPoints.Add(point);
        }

        return cornerHeldPoints;
    }
}
