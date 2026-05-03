package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class HoodCommand extends Command {

    private final HoodSubsystem hd;

    public HoodCommand(HoodSubsystem h) {
        this.hd = h;
        addRequirements(hd);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        hd.setHoodAngle(SmashAndShootConstants.HOOD_POSITION.in(Rotations));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
