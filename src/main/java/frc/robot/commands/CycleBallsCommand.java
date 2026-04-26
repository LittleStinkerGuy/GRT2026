package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants.HopperIntake;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.Constants.TowerConstants.TowerIntake;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;

public class CycleBallsCommand extends Command {
    private final FlywheelSubsystem flywheel;
    private final TowerRollersSubsystem tower;
    private final HopperSubsystem hopper;
    private final RollerIntakeSubsystem intakeRoller;

    private final Timer startUpTimer = new Timer();

    public CycleBallsCommand(FlywheelSubsystem flywheel, TowerRollersSubsystem tower, HopperSubsystem hopper, RollerIntakeSubsystem intakeRoller) {
        this.flywheel = flywheel;
        this.tower = tower;
        this.hopper = hopper;
        this.intakeRoller = intakeRoller;

        addRequirements(flywheel, tower, hopper, intakeRoller);
    }

    @Override
    public void initialize() {
        flywheel.flySpeed(1);
        tower.setTower(TowerIntake.BALLUP);
        hopper.setHopperState(HopperIntake.BALL_IN);

        startUpTimer.restart();
    }

    @Override
    public void execute() {
        flywheel.flySpeed(1);
        tower.setTower(TowerIntake.BALLUP);
        hopper.setHopperState(HopperIntake.BALL_IN);

        if (startUpTimer.hasElapsed(1)) {
            intakeRoller.runIn();
        } else {
            intakeRoller.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.flySpeed(0);
        tower.setManualControl(0);
        hopper.stop();
        intakeRoller.stop();
    }
}
