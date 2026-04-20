package frc.robot.commands;

import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterLearner;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;

/**
 * Close-range "smash and shoot" preset — runs ManualShooterSequence with the
 * hood angle and FlywheelSubsystem RPS from SmashAndShootConstants.
 */
public class SmashShot extends ManualShooterSequence {

    public SmashShot(
        FlywheelSubsystem fly,
        HoodSubsystem hood,
        TowerRollersSubsystem tower,
        HopperSubsystem hopper,
        PivotIntakeSubsystem pivotIntake,
        ShooterLearner learner) {
        super(
            fly,
            hood,
            tower,
            hopper,
            pivotIntake,
            learner,
            SmashAndShootConstants.HOOD_POSITION,
            SmashAndShootConstants.FLYWHEEL_RPS);
    }
}
