using System.Globalization;

namespace Racer3MotionUi.Models;

public readonly record struct CartesianPose(
    double X,
    double Y,
    double Z,
    double Roll = 0.0,
    double Pitch = 0.0,
    double Yaw = 0.0)
{
    public string ToCartesianArgument()
    {
        return string.Join(
            ",",
            Format(X),
            Format(Y),
            Format(Z),
            Format(Roll),
            Format(Pitch),
            Format(Yaw));
    }

    public string ToPreviewLine(int index)
    {
        return string.Create(
            CultureInfo.InvariantCulture,
            $"{index,2}: X={X,8:0.0000}  Y={Y,8:0.0000}  Z={Z,8:0.0000}  RPY=({Roll:0.###},{Pitch:0.###},{Yaw:0.###})");
    }

    private static string Format(double value)
    {
        return value.ToString("0.######", CultureInfo.InvariantCulture);
    }
}
