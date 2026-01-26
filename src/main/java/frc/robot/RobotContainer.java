// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// frc imports
import frc.robot.controllers.PS5DriveController;

// Subsystems
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Intake.RollerIntake;
import frc.robot.subsystems.Intake.PivotIntake;
import frc.robot.Constants.IntakeConstants;

//comands
 
import frc.robot.commands.intake.ManualIntakePivot;
import frc.robot.commands.intake.SetIntakePivot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private RollerIntake intakeSubsystem = new RollerIntake();
  private PivotIntake pivotIntake = new PivotIntake();

  private final Field2d m_field = new Field2d();




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    constructDriveController(); 
    constructMechController();
    configureBindings();

    CameraServer.startAutomaticCapture(); // start driver cam
    SmartDashboard.putData("Field", m_field);
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

    // Intake pivot controls

     mechController.square().onTrue(
      new SetIntakePivot(pivotIntake, IntakeConstants.STOWED_POS)
    );

    // Cross button -> Position 2
    mechController.cross().onTrue(
      new SetIntakePivot(pivotIntake, IntakeConstants.EXTENDED_POS)
    );


    /* Intake Controls - Hold button to run rollers */
    // R2 - intake in
    mechController.R2().whileTrue(
      new RunCommand(
        () -> intakeSubsystem.setDutyCycle(Constants.IntakeConstants.ROLLER_IN_SPEED),
        intakeSubsystem
      )
    ).onFalse(
      new RunCommand(
        () -> intakeSubsystem.stop(),
        intakeSubsystem
      ).withTimeout(0.02)
    );

    // L2 - intake out
    mechController.L2().whileTrue(
      new RunCommand(
        () -> intakeSubsystem.setDutyCycle(Constants.IntakeConstants.ROLLER_OUT_SPEED),
        intakeSubsystem
      )
    ).onFalse(
      new RunCommand(
        () -> intakeSubsystem.stop(),
        intakeSubsystem
      ).withTimeout(0.02)
    );

     // Pivot Configs: R2 for pivot up and L2 for pivot down
    pivotIntake.setDefaultCommand(
     new ManualIntakePivot(pivotIntake, () -> mechController.getR2Axis() - mechController.getL2Axis()
     )
   );



  }
  public void updateField() {
    // Get your robot's current pose from your drivetrain
    Pose2d robotPose = swerveSubsystem.getRobotPosition();
    m_field.setRobotPose(robotPose);
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
