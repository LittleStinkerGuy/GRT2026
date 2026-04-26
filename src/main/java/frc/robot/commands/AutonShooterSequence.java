package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;

/**
 * Manual shooter sequence - no auto-aim.
 * Uses fixed hood position and FlywheelSubsystem RPS from SmashAndShootConstants.
 * Waits for FlywheelSubsystem and hood to reach targets before feeding balls.
 */
public class AutonShooterSequence extends Command {

    private final FlywheelSubsystem fly;
    private final HoodSubsystem hd;
    private final TowerRollersSubsystem tower;
    private final HopperSubsystem hopper;
    private final PivotIntakeSubsystem pivotIntake;

    public AutonShooterSequence(
        FlywheelSubsystem fly,
        HoodSubsystem hood,
        TowerRollersSubsystem tower,
        HopperSubsystem hopper,
        PivotIntakeSubsystem pivotIntake) {
        this.fly = fly;
        this.hd = hood;
        this.tower = tower;
        this.hopper = hopper;
        this.pivotIntake = pivotIntake;

        addRequirements(fly, hood, tower, hopper);
    }

    @Override
    public void initialize() {
        // Start ramping FlywheelSubsystem and moving hood to position
        fly.shoot(SmashAndShootConstants.FLYWHEEL_RPS);
        hd.setHoodAngle(SmashAndShootConstants.HOOD_POSITION);
        pivotIntake.setPosition(IntakeConstants.PIVOT_IN_POS);
    }

    @Override
    public void execute() {
        // Keep commanding FlywheelSubsystem and hood targets
        fly.shoot(SmashAndShootConstants.FLYWHEEL_RPS);
        hd.setHoodAngle(SmashAndShootConstants.HOOD_POSITION);


        // Only feed balls when FlywheelSubsystem is at speed AND hood is at position
        if (/* fly.wantedVel() && hd.wantedAngl() */ true) {
            tower.setManualControl(SmashAndShootConstants.TOWER_DUTY_CYCLE);
            hopper.setDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        } else {
            tower.setManualControl(0);
            hopper.stop();
        }
        pivotIntake.setPosition(IntakeConstants.PIVOT_IN_POS);
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
