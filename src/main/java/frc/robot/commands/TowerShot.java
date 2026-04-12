package frc.robot.commands;

import frc.robot.Constants.TowerShootConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.shooterLearner;
import frc.robot.subsystems.shooter.towerRollers;

/**
 * Tower-shot preset — runs ManualShooterSequence with the hood angle and
 * flywheel RPS from TowerShootConstants. Placeholder values; tune on robot.
 */
public class TowerShot extends ManualShooterSequence {

    public TowerShot(
        flywheel fly,
        hood hood,
        towerRollers tower,
        HopperSubsystem hopper,
        PivotIntakeSubsystem pivotIntake,
        shooterLearner learner) {
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
