package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.HopperSubsystem;
import java.util.function.DoubleSupplier;

public class HopperManualCommand extends Command {
    private final HopperSubsystem HopperSubsystem;
    private final DoubleSupplier speedSupplier;

    public HopperManualCommand(HopperSubsystem subsystem, DoubleSupplier speedSupplier) {
        this.HopperSubsystem = subsystem;
        this.speedSupplier = speedSupplier;
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        
        if (Math.abs(speed) < 0.1) {
            speed = 0.0;
        }
        
        HopperSubsystem.setManualControl(speed);
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