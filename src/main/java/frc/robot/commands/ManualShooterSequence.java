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
public class ManualShooterSequence extends Command {

    private final flywheel fly;
    private final hood hd;
    private final towerRollers tower;
    private final HopperSubsystem hopper;
    private final PivotIntakeSubsystem pivotIntake;

    private final Timer pivotTimer = new Timer();
    private boolean pivotIsIn = true;

    public ManualShooterSequence(
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

        addRequirements(fly, hood, tower, hopper, pivotIntake);
    }

    @Override
    public void initialize() {
        // Start ramping flywheel and moving hood to position
        fly.shoot(SmashAndShootConstants.FLYWHEEL_RPS);
        hd.setHoodAngle(SmashAndShootConstants.HOOD_POSITION);
        // Start with pivot in
        pivotIsIn = true;
        pivotIntake.setPosition(IntakeConstants.PIVOT_IN_POS);
        pivotTimer.restart();
    }

    @Override
    public void execute() {
        // Keep commanding flywheel and hood targets
        fly.shoot(SmashAndShootConstants.FLYWHEEL_RPS);
        hd.setHoodAngle(SmashAndShootConstants.HOOD_POSITION);

        // Toggle pivot every 2 seconds
        if (pivotTimer.hasElapsed(2.0)) {
            pivotIsIn = !pivotIsIn;
            pivotTimer.restart();
        }
        pivotIntake.setPosition(pivotIsIn ? IntakeConstants.PIVOT_IN_POS : IntakeConstants.PIVOT_OUT_POS);

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
