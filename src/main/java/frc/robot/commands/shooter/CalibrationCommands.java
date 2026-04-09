package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.shooterLearner;
import frc.robot.subsystems.swerve.AimSubsystem;

/**
 * Operator-button commands for tuning the shooter interpolation table at runtime.
 *
 * Bind the up/down commands to controller buttons, drive to a known distance,
 * bump the offsets until shots score, then press the logPoint button to print
 * the (distance, rpm, hoodAngle) line. Copy those lines into Intertable.java
 * before the robot is power-cycled.
 */
public final class CalibrationCommands {

    private CalibrationCommands() {}

    public static Command rpmUp(shooterLearner learner) {
        return Commands.runOnce(learner::rpmUp);
    }

    public static Command rpmDown(shooterLearner learner) {
        return Commands.runOnce(learner::rpmDown);
    }

    public static Command hoodUp(shooterLearner learner) {
        return Commands.runOnce(learner::hoodUp);
    }

    public static Command hoodDown(shooterLearner learner) {
        return Commands.runOnce(learner::hoodDown);
    }

    public static Command resetOffsets(shooterLearner learner) {
        return Commands.runOnce(learner::reset);
    }

    /**
     * Logs the current calibration point: distance to the hub plus the
     * offset-adjusted rpm and hood angle that ManualShooterSequence is commanding.
     * Pulls base values from SmashAndShootConstants so the printed numbers match
     * what the robot is actually doing right now.
     */
    public static Command logPoint(shooterLearner learner, AimSubsystem aim) {
        return Commands.runOnce(() -> {
            double d = aim.getDistanceToHub();
            double rps = learner.getRPM(frc.robot.Constants.SmashAndShootConstants.FLYWHEEL_RPS);
            double angle = learner.getHoodAngle(frc.robot.Constants.SmashAndShootConstants.HOOD_POSITION);
            learner.log(d, rps, angle);
        });
    }
}
