package frc.robot.commands;

import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.shooterLearner;
import frc.robot.subsystems.shooter.towerRollers;

/**
 * Close-range "smash and shoot" preset — runs ManualShooterSequence with the
 * hood angle and flywheel RPS from SmashAndShootConstants.
 */
public class SmashShot extends ManualShooterSequence {

    public SmashShot(
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
            SmashAndShootConstants.HOOD_POSITION,
            SmashAndShootConstants.FLYWHEEL_RPS);
    }
}
