// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.VisionConstants;
// frc imports
import frc.robot.controllers.PS5DriveController;

// Subsystems
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.CameraConfig;

// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.function.BooleanSupplier;

import frc.robot.commands.swerve.RotateToAngleCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private boolean isCompetition = true;

  private PS5DriveController driveController;
  private CommandPS5Controller mechController;
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
    VisionConstants.cameraConfigs[0]
  );
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);

    constructDriveController(); 
    constructMechController();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named f`actories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
      /* Driving -- One joystick controls translation, the other rotation. If the robot-relative button is held down,
      * the robot is controlled along its own axes, otherwise controls apply to the field axes by default. If the
      * swerve aim button is held down, the robot will rotate automatically to always face a target, and only
      * translation will be manually controllable. */
    swerveSubsystem.setDefaultCommand(
      new RunCommand(() -> {
        swerveSubsystem.setDrivePowers(
          driveController.getForwardPower(),
          driveController.getLeftPower(),
          driveController.getRotatePower()
        );
        }, 
        swerveSubsystem
      )
    );
      
    driveController.getRelativeMode().whileTrue(
      new RunCommand(
        () -> {
          swerveSubsystem.setRobotRelativeDrivePowers(
            driveController.getForwardPower(),
            driveController.getLeftPower(),
            driveController.getRotatePower()
          );
          driveController.getRotatePower();
          }, swerveSubsystem)
    );


    /* Pressing the button resets the field axes to the current robot axes. */
    driveController.bindDriverHeadingReset(
      () ->{
        swerveSubsystem.resetDriverHeading();
      },
      swerveSubsystem
    );

    // Cancel rotate command if driver touches any stick
    BooleanSupplier driverInput = () ->
        Math.abs(driveController.getForwardPower()) > 0 ||
        Math.abs(driveController.getLeftPower()) > 0 ||
        Math.abs(driveController.getRotatePower()) > 0;

    // Triangle = rotate to 0°, Circle = rotate to 90°
    driveController.triangle().onTrue(new RotateToAngleCommand(swerveSubsystem, 0, driverInput));
    driveController.circle().onTrue(new RotateToAngleCommand(swerveSubsystem, 90, driverInput));

    // D-pad steer speed limiting (scales MotionMagic cruise velocity)
    // Up = 100%, Right = 75%, Down = 50%, Left = 25%
    new Trigger(() -> driveController.getPOV() == 0)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.setSteerSpeedLimit(1.0)));
    new Trigger(() -> driveController.getPOV() == 90)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.setSteerSpeedLimit(0.75)));
    new Trigger(() -> driveController.getPOV() == 180)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.setSteerSpeedLimit(0.50)));
    new Trigger(() -> driveController.getPOV() == 270)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.setSteerSpeedLimit(0.25)));
  }

  /**
   * Constructs the drive controller based on the name of the controller at port
   * 0
   */
  private void constructDriveController(){
    driveController = new PS5DriveController();
    driveController.setDeadZone(0.05);
  }

  /**
   * Constructs mech controller
   */
  private void constructMechController(){
    mechController = new CommandPS5Controller(1);
  }


}
 