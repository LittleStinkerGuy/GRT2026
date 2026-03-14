package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheel;

public class SpinFlywheelCommand extends Command {
    private final flywheel flySubsystem;
    private final double dutyCycle;

    public SpinFlywheelCommand(flywheel flySubsystem, double dutyCycle) {
        this.flySubsystem = flySubsystem;
        this.dutyCycle = dutyCycle;
        addRequirements(flySubsystem); // prevents other commands from controlling flywheel
    }

    @Override
    public void initialize() {
        flySubsystem.flySpeed(dutyCycle); // start spinning
    }

    @Override
    public void execute() {
        flySubsystem.flySpeed(dutyCycle); // keep it running
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

