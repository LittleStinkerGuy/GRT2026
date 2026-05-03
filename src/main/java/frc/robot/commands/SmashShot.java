package frc.robot.commands;

import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

/**
 * Close-range "smash and shoot" preset — runs ManualShooterSequence with the
 * hood angle and FlywheelSubsystem RPS from SmashAndShootConstants.
 */
public class SmashShot extends ManualShooterSequence {

    public SmashShot(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        PivotSubsystem pivot) {
        super(
            flywheel,
            hood,
            tower,
            hopper,
            pivot,
            SmashAndShootConstants.HOOD_POSITION,
            SmashAndShootConstants.FLYWHEEL_VELO);
    }
}
