package frc.robot.commands.shooter.towerrollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.Constants.TowerConstants.TowerIntake;

public class TowerRoll extends Command {

    private final TowerRollersSubsystem t;


    public TowerRoll(TowerRollersSubsystem b) {
        this.t = b;

        addRequirements(t);
    }

    @Override
    public void execute() {
        t.setManualControl(SmashAndShootConstants.TOWER_DUTY_CYCLE);
        // t.setTower(TOWER_INTAKE.BALL_UP);
        // if (fly.wantedVel() && hd.wantedAngl()) {
        // t.setTower(TOWER_INTAKE.BALL_UP);
        // } else {
        // t.setTower(TOWER_INTAKE.STOP);
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        t.setTower(TowerIntake.STOP);
    }
}
