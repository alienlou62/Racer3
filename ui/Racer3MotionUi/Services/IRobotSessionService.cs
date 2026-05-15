using System;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public interface IRobotSessionService
{
    bool IsRunning { get; }

    Task StartAsync(IProgress<ProcessOutputLine> output, CancellationToken cancellationToken = default);

    Task RequestStatusAsync(CancellationToken cancellationToken = default);

    Task StopMotionAsync(CancellationToken cancellationToken = default);

    Task ShutdownAsync(CancellationToken cancellationToken = default);
}
