// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.PivotIntake;


public class SetIntakePivot extends Command {
  private final PivotIntake pivotIntake;
  private final double targetAngle;

  /**
   * Creates a new SetPivotPosition command.
   *
   * @param pivotIntake The intake pivot subsystem
   * @param targetAngle Target angle in degrees
   */
  public SetIntakePivot(PivotIntake pivotIntake, double targetAngle) {
    this.pivotIntake = pivotIntake;
    this.targetAngle = targetAngle;
    addRequirements(pivotIntake);
  }

  @Override
  public void initialize() {
    pivotIntake.setAngle(targetAngle);
  }

  @Override
  public void execute() {
    // Continuously hold the position
    pivotIntake.setAngle(targetAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // manual control will take over when interrupted
  }
}

