package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;
import frc.robot.subsystems.vision.FuelDetectionSubsystem;

public class RunIntakeWithVisionCommand extends Command {
    private FuelDetectionSubsystem fuelDetection;
    private RollerIntakeSubsystem intake;

    private Distance closestDistance = Meters.of(-1);

    public RunIntakeWithVisionCommand(FuelDetectionSubsystem fuelDetectionSubsystem, RollerIntakeSubsystem rollerSubsystem) {
        this.fuelDetection = fuelDetectionSubsystem;
        this.intake = rollerSubsystem;

        addRequirements(fuelDetection, rollerSubsystem);
    }

    @Override
    public void initialize() {
        intake.stop();
    }

    @Override
    public void execute() {
        fuelDetection.getClosestDistance().ifPresent((distance) -> {
            closestDistance = distance;
        });

        if (closestDistance.lte(Meters.of(2)) && closestDistance.gte(Meters.of(.5))) {
            intake.runIn(); // TODO: SWITCH TO intake.runIn() or whatever it is now WHEN USING ACTUAL INTAKE SUBSYSTEM
        } else {
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
