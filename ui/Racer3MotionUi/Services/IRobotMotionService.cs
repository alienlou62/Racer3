using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Racer3MotionUi.Models;

namespace Racer3MotionUi.Services;

public interface IRobotMotionService
{
    IReadOnlyList<MotionCommand> BuildTraceCommands(ShapeTracePlan plan, RobotMotionOptions options);

    MotionCommand BuildXboxControllerJogCommand();

    Task<MotionExecutionResult> RunXboxControllerJogAsync(
        IProgress<ProcessOutputLine> output,
        CancellationToken cancellationToken);

    Task<MotionExecutionResult> TraceShapeAsync(
        ShapeTracePlan plan,
        RobotMotionOptions options,
        IProgress<ProcessOutputLine> output,
        CancellationToken cancellationToken);
}
