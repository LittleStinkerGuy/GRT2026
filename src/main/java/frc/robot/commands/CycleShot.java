package frc.robot.commands;

import frc.robot.Constants.CycleShooterConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.shooterLearner;
import frc.robot.subsystems.shooter.towerRollers;
import frc.robot.subsystems.hopper.HopperSubsystem;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Manual shooter sequence - no auto-aim.
 * Hood position and flywheel RPS are passed in via the constructor so the same
 * sequence can be reused for different shot types (smash, cycle, etc.). Pivot
 * timing comes from SmashAndShootConstants.
 *
 * The shooterLearner offsets are applied on every loop, so live tuning via the
 * dashboard or operator buttons takes effect mid-shot.
 */
public class CycleShot extends Command {

    private final flywheel fly;
    private final hood hd;
    private final towerRollers tower;
    private final HopperSubsystem hopper;
    private final PivotIntakeSubsystem pivotIntake;

    private final DoubleSupplier flywheelVelo;

    private final Timer pivotTimer = new Timer();
    private boolean pivotIsIn = true;
    private boolean initialDelayDone = false;

    public CycleShot(
        flywheel fly,
        hood hood,
        towerRollers tower,
        HopperSubsystem hopper,
        PivotIntakeSubsystem pivotIntake,
        DoubleSupplier flyWheelVeloSupplier) {
        this.fly = fly;
        this.hd = hood;
        this.tower = tower;
        this.hopper = hopper;
        this.pivotIntake = pivotIntake;
        this.flywheelVelo = flyWheelVeloSupplier;

        addRequirements(fly, hood, tower, hopper, pivotIntake);
    }

    @Override
    public void initialize() {
        // Start ramping flywheel and moving hood to position
        fly.shoot(flywheelVelo.getAsDouble());
        hd.setHoodAngle(CycleShooterConstants.HOOD_POSITION);
        // Start with pivot out, wait the initial-delay before first toggle
        pivotIsIn = false;
        initialDelayDone = false;
        pivotIntake.setPosition(IntakeConstants.PIVOT_OUT_POS);
        pivotTimer.restart();
    }

    @Override
    public void execute() {
        // Keep commanding flywheel and hood targets (with live operator offsets)
        fly.shoot(flywheelVelo.getAsDouble());
        hd.setHoodAngle(CycleShooterConstants.HOOD_POSITION);

        if (!initialDelayDone) {
            if (pivotTimer.hasElapsed(SmashAndShootConstants.INITIAL_DELAY_SECONDS)) {
                initialDelayDone = true;
                pivotIsIn = true;
                pivotTimer.restart();
            }
        } else if (pivotTimer.hasElapsed(SmashAndShootConstants.TOGGLE_INTERVAL_SECONDS)) {
            pivotIsIn = !pivotIsIn;
            pivotTimer.restart();
        }
        pivotIntake.setPosition(pivotIsIn ? IntakeConstants.PIVOT_MID_UPPER : IntakeConstants.PIVOT_MID_LOWER);

        // Only feed balls when flywheel is at speed AND hood is at position
        if (/* fly.wantedVel() && hd.wantedAngl() */ true) {
            tower.setManualControl(SmashAndShootConstants.TOWER_DUTY_CYCLE);
            hopper.setManualControl(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        } else {
            tower.setManualControl(0);
            hopper.setManualControl(0);
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
        hopper.setManualControl(0);
        pivotIntake.setPosition(IntakeConstants.PIVOT_OUT_POS);
    }
}
