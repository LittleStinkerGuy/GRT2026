package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants.HopperIntake;
import frc.robot.subsystems.hopper.HopperSubsystem;
import java.util.function.DoubleSupplier;

public class HopperManualCommand extends Command {
    private final HopperSubsystem hopper;
    private final DoubleSupplier speedSupplier;

    public HopperManualCommand(HopperSubsystem hopper, DoubleSupplier speedSupplier) {
        this.hopper = hopper;
        this.speedSupplier = speedSupplier;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();

        if (Math.abs(speed) < 0.1) {
            speed = 0.0;
        }

        hopper.setManualControl(speed);
    }

    @Override
    public void end(boolean interrupted) {
        hopper.setHopper(HopperIntake.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
