package frc.robot.commands.shooter.towerRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.towerRollers;
import frc.robot.Constants.TowerConstants.TOWER_INTAKE;

public class towerRoll extends Command {

    private towerRollers t;

    public towerRoll(towerRollers b) {
        t = b;
        addRequirements(t);
    }

    @Override
    public void execute() {
        t.setTower(TOWER_INTAKE.BALLUP);
    }

    @Override
    public void end(boolean interrupted) {
        t.setTower(TOWER_INTAKE.STOP);
    }
}
