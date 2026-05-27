using System;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public interface IRobotSessionService
{
    bool IsRunning { get; }

    Task StartAsync(IProgress<ProcessOutputLine> output, CancellationToken cancellationToken);

    Task RequestStatusAsync(CancellationToken cancellationToken);

    Task StopMotionAsync(CancellationToken cancellationToken);

    Task StartCartesianJogAsync(string direction, double speedMetersPerSecond, CancellationToken cancellationToken);

    Task StopCartesianJogAsync(string reason, CancellationToken cancellationToken);

    Task<MotionExecutionResult> TraceShapeAsync(
        ShapeTracePlan plan,
        RobotMotionOptions options,
        IProgress<ProcessOutputLine> output,
        CancellationToken cancellationToken);

    Task ShutdownAsync(CancellationToken cancellationToken);
}
