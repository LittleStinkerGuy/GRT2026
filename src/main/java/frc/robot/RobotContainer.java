// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// frc imports
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.climb.ClimbSubsystem;
// Subsystems
import frc.robot.subsystems.swerve.SwerveSubsystem;

import com.ctre.phoenix6.CANBus;

// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private CANBus canivore = new CANBus("can");

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private boolean isCompetition = true;

  private PS5DriveController driveController;
  private CommandPS5Controller mechController;
  // private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem(canivore);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    constructDriveController();
    constructMechController();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named f`actories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*
     * Driving -- One joystick controls translation, the other rotation. If the
     * robot-relative button is held down,
     * the robot is controlled along its own axes, otherwise controls apply to the
     * field axes by default. If the
     * swerve aim button is held down, the robot will rotate automatically to always
     * face a target, and only
     * translation will be manually controllable.
     */
    // swerveSubsystem.setDefaultCommand(
    // new RunCommand(() -> {
    // swerveSubsystem.setDrivePowers(
    // driveController.getForwardPower(),
    // driveController.getLeftPower(),
    // driveController.getRotatePower()
    // );
    // },
    // swerveSubsystem
    // )
    // );

    // driveController.getRelativeMode().whileTrue(
    // new RunCommand(
    // () -> {
    // swerveSubsystem.setRobotRelativeDrivePowers(
    // driveController.getForwardPower(),
    // driveController.getLeftPower(),
    // driveController.getRotatePower()
    // );
    // driveController.getRotatePower();
    // }, swerveSubsystem)
    // );

    // /* Pressing the button resets the field axes to the current robot axes. */
    // driveController.bindDriverHeadingReset(
    // () ->{
    // swerveSubsystem.resetDriverHeading();
    // },
    // swerveSubsystem
    // );

    var crossTrigger = mechController.cross();
    var triangleTrigger = mechController.triangle();
    crossTrigger.onTrue(m_ClimbSubsystem.climbDown(() -> crossTrigger.getAsBoolean()));
    triangleTrigger.onTrue(m_ClimbSubsystem.climbUp(() -> triangleTrigger.getAsBoolean()));

    m_ClimbSubsystem.setDefaultCommand(Commands.run(() -> {
      var armSpeed = -mechController.getLeftY();
      double winchSpeed = 0;

      if (mechController.povUp().getAsBoolean()) {
        winchSpeed++;
      }
      if (mechController.povDown().getAsBoolean()) {
        winchSpeed--;
      }
      m_ClimbSubsystem.setArmSpeed(armSpeed);
      m_ClimbSubsystem.setWinchSpeed(winchSpeed);
    }, m_ClimbSubsystem));
  }

  /**
   * Constructs the drive controller based on the name of the controller at port
   * 0
   */
  private void constructDriveController() {
    driveController = new PS5DriveController();
    driveController.setDeadZone(0.05);
  }

  /**
   * Constructs mech controller
   */
  private void constructMechController() {
    mechController = new CommandPS5Controller(1);
  }

}
