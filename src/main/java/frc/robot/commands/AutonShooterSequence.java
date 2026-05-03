package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

/**
 * Manual shooter sequence - no auto-aim.
 * Uses fixed hood position and FlywheelSubsystem RPS from SmashAndShootConstants.
 * Waits for FlywheelSubsystem and hood to reach targets before feeding balls.
 */
public class AutonShooterSequence extends Command {

    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final TowerSubsystem tower;
    private final HopperSubsystem hopper;
    private final PivotSubsystem pivot;

    public AutonShooterSequence(
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        TowerSubsystem tower,
        HopperSubsystem hopper,
        PivotSubsystem pivot) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.tower = tower;
        this.hopper = hopper;
        this.pivot = pivot;

        addRequirements(flywheel, hood, tower, hopper);
    }

    @Override
    public void initialize() {
        // Start ramping FlywheelSubsystem and moving hood to position
        flywheel.setVelocity(SmashAndShootConstants.FLYWHEEL_VELO);
        hood.setHoodAngle(SmashAndShootConstants.HOOD_POSITION.in(Rotations));
        pivot.setPosition(IntakeConstants.PIVOT_IN_POS);
    }

    @Override
    public void execute() {
        // Keep commanding FlywheelSubsystem and hood targets
        flywheel.setVelocity(SmashAndShootConstants.FLYWHEEL_VELO);
        hood.setHoodAngle(SmashAndShootConstants.HOOD_POSITION.in(Rotations));


        // Only feed balls when FlywheelSubsystem is at speed AND hood is at position
        if (/* fly.wantedVel() && hd.wantedAngl() */ true) {
            tower.setDutyCycle(SmashAndShootConstants.TOWER_DUTY_CYCLE);
            hopper.setDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        } else {
            tower.stop();
            hopper.stop();
        }
        pivot.setPosition(IntakeConstants.PIVOT_IN_POS);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        hood.setHoodAngle(0);
        tower.stop();
        hopper.stop();
    }
}
