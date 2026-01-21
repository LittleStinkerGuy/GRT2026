// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake.PivotIntake;


public class SetIntakePivot extends InstantCommand {
  /**
   * Creates a new SetPivotPosition command.
   *
   * @param PivotIntake The intake pivot subsystem
   * @param targetAngle Target angle in degrees
   */
  public SetIntakePivot(PivotIntake pivotIntake, double targetAngle) {
    super(
      () -> {
        pivotIntake.setAngle(targetAngle);
      },
      pivotIntake
    );
  }
}