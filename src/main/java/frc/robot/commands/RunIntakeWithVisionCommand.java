package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants.HopperIntake;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;
import frc.robot.subsystems.vision.FuelDetectionSubsystem;
import java.util.function.DoubleSupplier;

public class RunIntakeWithVisionCommand extends Command {
    private FuelDetectionSubsystem fuelDetection;
    private HopperSubsystem intake;

    private Distance closestDistance = Meters.of(-1);

    public RunIntakeWithVisionCommand(FuelDetectionSubsystem fuelDetectionSubsystem, HopperSubsystem rollerSubsystem) {
        this.fuelDetection = fuelDetectionSubsystem;
        this.intake = rollerSubsystem;

        addRequirements(fuelDetection, rollerSubsystem);
    }

    @Override
    public void initialize() {
        intake.setManualControl(0);
    }

    @Override
    public void execute() {
        fuelDetection.getClosestDistance().ifPresent((distance) -> {
            closestDistance = distance;
        });

        if (closestDistance.lte(Meters.of(2)) && closestDistance.gte(Meters.of(.5))) {
            intake.setHopper(HopperIntake.BALL_IN); // TODO: SWITCH TO intake.runIn() or whatever it is now WHEN USING ACTUAL INTAKE SUBSYSTEM
        } else {
            intake.setManualControl(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setManualControl(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
