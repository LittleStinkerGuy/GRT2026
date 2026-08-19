package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.CycleShooterConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.DoubleSupplier;

/**
 * Cycle shot - no auto-aim and no pivot movement. Hood position comes from
 * CycleShooterConstants; the FlywheelSubsystem RPS is supplied so live tuning via
 * the dashboard or operator buttons takes effect mid-shot.
 */
public class CycleShot extends ParallelCommandGroup {

    public CycleShot(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        DoubleSupplier flyWheelVeloSupplier,
        DoubleSupplier hoodPosSupplier) {
        super(

            flywheel.setFlywheelVelocity(() -> (CycleShooterConstants.FLYWHEEL_VELO.in(RotationsPerSecond))),
            hood.holdPositionThenHide(() -> (CycleShooterConstants.HOOD_POSITION.in(Rotations))),
            tower.runTowerDutyCycle(SmashAndShootConstants.TOWER_DUTY_CYCLE),
            hopper.runHopperDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE));
    }
}
