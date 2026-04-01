package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.Constants.HopperConstants.HOPPER_INTAKE;
import frc.robot.Constants.TowerConstants.TOWER_INTAKE;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;

public class cycleBallsCommand extends Command {
    private final flywheel flywheel;
    private final towerRollers tower;
    private final HopperSubsystem hopper;
    private final RollerIntakeSubsystem intakeRoller;

    private final Timer startUpTimer = new Timer();

    public cycleBallsCommand(flywheel flywheel, towerRollers tower, HopperSubsystem hopper, RollerIntakeSubsystem intakeRoller) {
        this.flywheel = flywheel;
        this.tower = tower;
        this.hopper = hopper;
        this.intakeRoller = intakeRoller;

        addRequirements(flywheel, tower, hopper, intakeRoller);
    }

    @Override
    public void initialize() {
        flywheel.flySpeed(1);
        tower.setTower(TOWER_INTAKE.BALLUP);
        hopper.setHopper(HOPPER_INTAKE.BALLIN);

        startUpTimer.restart();
    }

    @Override
    public void execute() {
        flywheel.flySpeed(1);
        tower.setTower(TOWER_INTAKE.BALLUP);
        hopper.setHopper(HOPPER_INTAKE.BALLIN);

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
        hopper.setManualControl(0);
        intakeRoller.stop();
    }
}
