package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheel;

public class SpinFlywheelCommand extends Command {
    private final flywheel flySubsystem;
    private final double rps;

    public SpinFlywheelCommand(flywheel flySubsystem, double rps) {
        this.flySubsystem = flySubsystem;
        this.rps = rps;
        addRequirements(flySubsystem); // prevents other commands from controlling flywheel
    }

    @Override
    public void initialize() {
        flySubsystem.shoot(rps); // start spinning
    }

    @Override
    public void execute() {
        flySubsystem.shoot(rps); // keep it running
    }

    @Override
    public void end(boolean interrupted) {
        flySubsystem.dontShoot(); // stop flywheel when command ends
    }

    @Override
    public boolean isFinished() {
        return false; // never ends on its own
    }
}

