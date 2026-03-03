package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.Vision.FuelDetectionSubsystem;

// WARNING: THIS COMMAND CURRENTLY USES THE HOPPER INSTEAD OF THE INTAKE
// THIS IS A TEMPORARY WORKAROUND WHILE THE ACTUAL INTAKE IS NOT WORKING
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
    intake.stop();
  }

  @Override
  public void execute() {
    fuelDetection.getClosestDistance().ifPresent((distance) -> {
      closestDistance = distance;
    });

    if (closestDistance.lte(Meters.of(2)) && closestDistance.gte(Meters.of(.5))) {
      intake.runForward(); // TODO: SWITCH TO intake.runIn() WHEN USING ACTUAL INTAKE SUBSYSTEM
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
