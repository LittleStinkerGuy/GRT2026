// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.VisionConstants;
import frc.robot.commands.allign.AlignToHubCommand;
import frc.robot.commands.allign.RotateToAngleCommand;
// frc imports
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.climb.ClimbSubsystem;
// Subsystems
import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.Vision.VisionSubsystem;
// import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
// import frc.robot.Constants.IntakeConstants;

// Commands
import frc.robot.commands.intake.ManualIntakePivotCommand;

import com.ctre.phoenix6.CANBus;

// import frc.robot.commands.intake.SetIntakePivotCommand;
// import frc.robot.commands.hopper.HopperSetRPMCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;



import com.ctre.phoenix6.CANBus;

// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private PS5DriveController driveController;
  private CommandPS5Controller mechController;
  private final CANBus swerveCAN = new CANBus(Constants.Swerve_CAN_BUS);
  private final CANBus mechCAN = new CANBus(Constants.Mech_CAN_BUS);

  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem(swerveCAN);

  private final RollerIntakeSubsystem intakeSubsystem = new RollerIntakeSubsystem(mechCAN);
  private final PivotIntakeSubsystem pivotIntake = new PivotIntakeSubsystem(mechCAN);
  private final HopperSubsystem HopperSubsystem = new HopperSubsystem(mechCAN);
  private final Field2d m_field = new Field2d();
  private ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem(canivore);

  // private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
  //   VisionConstants.cameraConfigs[0]
  // );
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);

    constructDriveController();
    constructMechController();
    configureBindings();
    configureAutoChooser();

    CameraServer.startAutomaticCapture(); // start driver cam
    SmartDashboard.putData("Field", m_field);
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
    swerveSubsystem.setDefaultCommand(
        new RunCommand(() -> {
          swerveSubsystem.setDrivePowers(
              driveController.getForwardPower(),
              driveController.getLeftPower(),
              driveController.getRotatePower());
        },
            swerveSubsystem));

    driveController.getRelativeMode().whileTrue(
        new RunCommand(
            () -> {
              swerveSubsystem.setRobotRelativeDrivePowers(
                  driveController.getForwardPower(),
                  driveController.getLeftPower(),
                  driveController.getRotatePower());
              driveController.getRotatePower();
            }, swerveSubsystem));

    /* Pressing the button resets the field axes to the current robot axes. */
    driveController.bindDriverHeadingReset(
        () -> {
          swerveSubsystem.resetDriverHeading();
        },
        swerveSubsystem);

    // bind semi auto commands
    var crossTrigger = mechController.cross();
    var triangleTrigger = mechController.triangle();
    crossTrigger.onTrue(m_ClimbSubsystem.climbDown(() -> crossTrigger.getAsBoolean()));
    triangleTrigger.onTrue(m_ClimbSubsystem.climbUp(() -> triangleTrigger.getAsBoolean()));

    // Manual control with d-pad for winch and left stick for arm
    m_ClimbSubsystem.setDefaultCommand(Commands.run(() -> {
      var armDutyCycle = mechController.getLeftY();
      double winchDutyCycle = 0;

      if (mechController.povUp().getAsBoolean()) {
        winchDutyCycle++;
      }
      if (mechController.povDown().getAsBoolean()) {
        winchDutyCycle--;
      }
      m_ClimbSubsystem.setArmDutyCycle(armDutyCycle);
      m_ClimbSubsystem.setWinchDutyCycle(winchDutyCycle);
    }, m_ClimbSubsystem));

    // Cancel rotate command if driver touches any stick
    BooleanSupplier driverInput = () ->
        Math.abs(driveController.getForwardPower()) > 0 ||
        Math.abs(driveController.getLeftPower()) > 0 ||
        Math.abs(driveController.getRotatePower()) > 0;

    // Triangle = rotate to 0°, Circle = rotate to 90°
    driveController.triangle().onTrue(new RotateToAngleCommand(swerveSubsystem, 0, driverInput));
    driveController.circle().onTrue(new RotateToAngleCommand(swerveSubsystem, 90, driverInput));

    // L1 = align to hub
    new Trigger(driveController::getLeftBumper).onTrue(AlignToHubCommand.create(swerveSubsystem, driverInput));

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

  /**
   * Config the autonomous command chooser
   */
  private void configureAutoChooser() {
    // Add auton here
    autoChooser.setDefaultOption("Do Nothing", null);

    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
 