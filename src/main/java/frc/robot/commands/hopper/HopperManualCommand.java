package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.HopperMotor;
import java.util.function.DoubleSupplier;

public class HopperManualCommand extends Command {
    private final HopperMotor hopperMotor;
    private final DoubleSupplier speedSupplier;

    public HopperManualCommand(HopperMotor subsystem, DoubleSupplier speedSupplier) {
        this.hopperMotor = subsystem;
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
        
        hopperMotor.setManualControl(speed);
    }
    
    @Override
    public void end(boolean interrupted) {
        hopperMotor.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}