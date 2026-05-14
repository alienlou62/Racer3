using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public interface IShapePathPlanner
{
    ShapeTracePlan CreatePlan(ShapeKind shape, CartesianPose center, double sizeMeters);
}
