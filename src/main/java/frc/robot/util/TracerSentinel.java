package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Empty subsystem used purely as a marker for {@link LoggedTracer}. Construct this BEFORE any other
 * subsystem so it registers first with the CommandScheduler — its {@code periodic()} then runs
 * before every other subsystem's, capturing pre-subsystem scheduler overhead under
 * {@code SchedulerStartMS}.
 */
public class TracerSentinel extends SubsystemBase {
    @Override
    public void periodic() {
        LoggedTracer.record("SchedulerStart");
    }
}
