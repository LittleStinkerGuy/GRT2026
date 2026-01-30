// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

// --- Set position command (commented out for now - using manual control) ---
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Intake.PivotIntake;
//
// public class SetIntakePivotCommand extends Command {
//   private final PivotIntake pivotIntake;
//   private final double targetAngle;
//
//   public SetIntakePivotCommand(PivotIntake pivotIntake, double targetAngle) {
//     this.pivotIntake = pivotIntake;
//     this.targetAngle = targetAngle;
//     addRequirements(pivotIntake);
//   }
//
//   @Override
//   public void initialize() {
//     pivotIntake.setAngle(targetAngle);
//   }
//
//   @Override
//   public void execute() {
//     pivotIntake.setAngle(targetAngle);
//   }
//
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
//
//   @Override
//   public void end(boolean interrupted) {
//   }
// }
