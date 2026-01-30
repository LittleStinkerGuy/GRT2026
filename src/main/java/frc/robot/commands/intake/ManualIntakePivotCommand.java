// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import java.util.function.DoubleSupplier;

public class ManualIntakePivotCommandCommand extends Command {
  private final PivotIntakeSubsystem pivotIntake;
  private final DoubleSupplier speedSupplier;

   /**
   * Creates a new ManualPivot command
   *
   * @param pivotIntake Pivot subsystem
   * @param speedSupplier A supplier returning the desired speed (-1.0 to 1.0), in this case its R2 and L2
   *
   */
  public ManualIntakePivotCommandCommand(PivotIntakeSubsystem pivotIntake, DoubleSupplier speedSupplier) {
    this.pivotIntake = pivotIntake;
    this.speedSupplier = speedSupplier;
    addRequirements(pivotIntake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Get the current speed value from R2 and L2
    double speedValue = speedSupplier.getAsDouble();
    pivotIntake.setManualSpeed(speedValue * IntakeConstants.MANUAL_PIVOT_SPEED);

  }

  @Override
  public void end(boolean interrupted) {
    pivotIntake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}