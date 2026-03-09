package frc.robot.commands.shooter.towerRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.towerRollers;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.Constants.TowerConstants.TOWER_INTAKE;

public class towerRoll extends Command {

    private final towerRollers t;
    private final flywheel fly;

    public towerRoll(towerRollers b, flywheel fly) {
        this.t = b;
        this.fly = fly;
        addRequirements(t);
    }

    @Override
    public void execute() {
        // Only run tower when flywheel is up to speed
        if (fly.wantedVel()) {
            t.setTower(TOWER_INTAKE.BALLUP);
        } else {
            t.setTower(TOWER_INTAKE.STOP);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        t.setTower(TOWER_INTAKE.STOP);
    }
}
