package frc.robot.commands;

import frc.robot.Constants.TowerShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterLearner;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;

/**
 * Tower-shot preset — runs ManualShooterSequence with the hood angle and
 * FlywheelSubsystem RPS from TowerShootConstants. Placeholder values; tune on robot.
 */
public class TowerShot extends ManualShooterSequence {

    public TowerShot(
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
            TowerShootConstants.HOOD_POSITION,
            TowerShootConstants.FLYWHEEL_RPS);
    }
}
