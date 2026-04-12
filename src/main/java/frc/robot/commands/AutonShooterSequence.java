package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;
import frc.robot.subsystems.hopper.HopperSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual shooter sequence - no auto-aim.
 * Uses fixed hood position and flywheel RPS from SmashAndShootConstants.
 * Waits for flywheel and hood to reach targets before feeding balls.
 */
public class AutonShooterSequence extends Command {

    private final flywheel fly;
    private final hood hd;
    private final towerRollers tower;
    private final HopperSubsystem hopper;
    private final PivotIntakeSubsystem pivotIntake;

    public AutonShooterSequence(
        flywheel fly,
        hood hood,
        towerRollers tower,
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
        // Start ramping flywheel and moving hood to position
        fly.shoot(SmashAndShootConstants.FLYWHEEL_RPS);
        hd.setHoodAngle(SmashAndShootConstants.HOOD_POSITION);
        pivotIntake.setPosition(IntakeConstants.PIVOT_IN_POS);
    }

    @Override
    public void execute() {
        // Keep commanding flywheel and hood targets
        fly.shoot(SmashAndShootConstants.FLYWHEEL_RPS);
        hd.setHoodAngle(SmashAndShootConstants.HOOD_POSITION);


        // Only feed balls when flywheel is at speed AND hood is at position
        if (/* fly.wantedVel() && hd.wantedAngl() */ true) {
            tower.setManualControl(SmashAndShootConstants.TOWER_DUTY_CYCLE);
            hopper.setManualControl(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        } else {
            tower.setManualControl(0);
            hopper.setManualControl(0);
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
        hopper.setManualControl(0);
    }
}
