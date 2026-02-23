package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.HopperSubsystem;

public class HopperSetRPMCommand extends Command {
    private final HopperSubsystem HopperSubsystem;

    public HopperSetRPMCommand(HopperSubsystem subsystem) {
        this.HopperSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        HopperSubsystem.spinAtTargetRPM();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        HopperSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
