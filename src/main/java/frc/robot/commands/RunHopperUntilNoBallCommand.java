package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.Vision.FuelDetectionSubsystem;

public class RunHopperUntilNoBallCommand extends Command {
    private final FuelDetectionSubsystem fuelDetection;
    private final HopperSubsystem hopper;

    private boolean finished = false;

    public RunHopperUntilNoBallCommand(FuelDetectionSubsystem fuelDetection, HopperSubsystem hopper) {
        this.fuelDetection = fuelDetection;
        this.hopper = hopper;

        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        finished = false;
        if (!fuelDetection.isBallDetected()) {
            finished = true;
        }
    }

    @Override
    public void execute() {
        hopper.runForward();
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stop();
    }

    @Override
    public boolean isFinished() {
        return finished || !fuelDetection.isBallDetected();
    }
}
