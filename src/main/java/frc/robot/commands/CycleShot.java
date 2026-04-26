package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CycleShooterConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Manual shooter sequence - no auto-aim.
 * Hood position and FlywheelSubsystem RPS are passed in via the constructor so the same
 * sequence can be reused for different shot types (smash, cycle, etc.). Pivot
 * timing comes from SmashAndShootConstants.
 *
 * The shooterLearner offsets are applied on every loop, so live tuning via the
 * dashboard or operator buttons takes effect mid-shot.
 */
public class CycleShot extends Command {

    private final FlywheelSubsystem fly;
    private final HoodSubsystem hd;
    private final TowerRollersSubsystem tower;
    private final HopperSubsystem hopper;

    private final DoubleSupplier flywheelVelo;


    public CycleShot(
        FlywheelSubsystem fly,
        HoodSubsystem hood,
        TowerRollersSubsystem tower,
        HopperSubsystem hopper,
        DoubleSupplier flyWheelVeloSupplier) {
        this.fly = fly;
        this.hd = hood;
        this.tower = tower;
        this.hopper = hopper;
        this.flywheelVelo = flyWheelVeloSupplier;

        addRequirements(fly, hood, tower, hopper);
    }

    @Override
    public void initialize() {
        // Start ramping FlywheelSubsystem and moving hood to position
        fly.shoot(flywheelVelo.getAsDouble());
        hd.setHoodAngle(CycleShooterConstants.HOOD_POSITION);
    }

    @Override
    public void execute() {
        // Keep commanding FlywheelSubsystem and hood targets (with live operator offsets)
        fly.shoot(flywheelVelo.getAsDouble());
        hd.setHoodAngle(CycleShooterConstants.HOOD_POSITION);


        // Only feed balls when FlywheelSubsystem is at speed AND hood is at position
        if (/* fly.wantedVel() && hd.wantedAngl() */ true) {
            tower.setManualControl(SmashAndShootConstants.TOWER_DUTY_CYCLE);
            hopper.setDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        } else {
            tower.setManualControl(0);
            hopper.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        fly.dontShoot();
        hd.setHoodAngle(0);
        tower.setManualControl(0);
        hopper.stop();
    }
}
