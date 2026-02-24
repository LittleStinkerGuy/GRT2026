// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.util.PolynomialRegression;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // ==================== GLOBAL ====================
  public static final String Swerve_CAN_BUS = "swerveCAN";
  public static final String Mech_CAN_BUS = "mechCAN";

  // Subsystem Enable/Disable
  public static final boolean SWERVE_ENABLED = true;
  public static final boolean MECH_ENABLED = true;
  // ==================== SHOOTER ====================
  public static class ShooterConstants {
    // CAN IDs (: Hood=16, Flywheel=17, Flywheel Encoder=18)
    public static final int HOOD_CAN_ID = 16;
    public static final int FLYWHEEL_CAN_ID = 17;
    public static final int FLYWHEEL_ENCODER_ID = 18;

    // Physics constants
    public static final double g = 9.8;
    public static final double tan75 = 3.73205;
    public static final double cos75 = 0.258819;
    public static final double height = 1.83;
    public static final double radius = 0.0508;

    // Gear ratios
    public static final double GEAR_RATIO_FLYWHEEL = 1.5;
    public static final double GEAR_RATIO_HOOD = 50;

    // Angles
    public static final double UPPER_ANGLE = 0.45;
    public static final double LOWER_ANGLE = -1.25;
    public static final double MAX_VELO = 1;
    public static final double INIT_HOOD_ANGLE = -1.25;
    public static final double HOOD_MAGNET_OFFSET = 0.5;

    // Hub position
    public static final Pose2d HUB_POS = new Pose2d(4.03479, 4.0288, null);

    // Manual control
    public static final double HOOD_MANUAL_SPEED = 0.15;
    public static final double FLYWHEEL_MANUAL_SPEED = 0.5;
  }

  // ==================== DRIVETRAIN ====================

  public static class SwerveDriveConstants {
    // Motor Configuration
    public static final double DRIVE_PEAK_CURRENT = 80;
    public static final double DRIVE_RAMP_RATE = 0;

    // Current Limits (optimized for Kraken motors)
    public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 50;     // Prevents brownouts
    public static final double DRIVE_STATOR_CURRENT_LIMIT = 100;    // Allows burst torque for acceleration
    public static final boolean DRIVE_CURRENT_LIMIT_ENABLE = true;

    // Physical Measurements
    public static final double DRIVE_WHEEL_RADIUS = 2.0; // inches
    public static final double DRIVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(2.0 * Math.PI * DRIVE_WHEEL_RADIUS);
    public static final double DRIVE_GEAR_REDUCTION = 33.0 / 4.0; // 8.25:1
  }

  public static class SwerveSteerConstants {
    // Motor Configuration
    public static final double STEER_PEAK_CURRENT = 40;
    public static final double STEER_RAMP_RATE = 0;

    // Current Limits (optimized for Kraken motors - steer needs less current)
    public static final double STEER_SUPPLY_CURRENT_LIMIT = 25;     // Prevents brownouts
    public static final double STEER_STATOR_CURRENT_LIMIT = 50;     // Sufficient for steering
    public static final boolean STEER_CURRENT_LIMIT_ENABLE = true;

    // Physical Measurements
    public static final double STEER_GEAR_REDUCTION = 160.0 / 7.0; // ~22.86:1
    public static final double STEER_FREE_SPEED_RPM = 7530.0; // Kraken X44

    // Motion Magic
    public static final double STEER_MAX_VELOCITY = STEER_FREE_SPEED_RPM / STEER_GEAR_REDUCTION / 60.0; // ~5.49 rot/sec
    public static final double STEER_MAX_ACCELERATION = STEER_MAX_VELOCITY * 10.0; // ~54.9 rot/sec^2
    public static final double STEER_CRUISE_VELOCITY = STEER_MAX_VELOCITY;
    public static final double STEER_ACCELERATION = STEER_MAX_ACCELERATION;
  }

  public static class SwerveConstants {
    // Drive PID (Velocity Control)
    public static final double[] DRIVE_P = {9.5, 9.5, 9.5, 9.5};
    public static final double[] DRIVE_I = {0, 0, 0, 0};
    public static final double[] DRIVE_D = {0.1, 0.1, 0.1, 0.1};
    public static final double[] DRIVE_S = {0.5, 0.5, 0.5, 0.5};
    public static final double[] DRIVE_V = {0.12, 0.12, 0.12, 0.12};

    // Steer PID (Position Control)
    public static final double[] STEER_P = {190, 190, 190, 190};
    public static final double[] STEER_I = {0, 0, 0, 0};
    public static final double[] STEER_D = {7, 7, 7, 7};
    public static final double[] STEER_S = {1, 1, 1, 1};

    // ID
    public static final int PigeonID = 24;
    // Module CAN IDs and Offsets (per README)
    public static final int FL_DRIVE = 0;
    public static final int FL_STEER = 1;
    public static final int FL_ENCODER = 8;
    public static final double FL_OFFSET = 0;

    public static final int FR_DRIVE = 2;
    public static final int FR_STEER = 3;
    public static final int FR_ENCODER = 9;
    public static final double FR_OFFSET = 0;

    public static final int BL_DRIVE = 4;
    public static final int BL_STEER = 5;
    public static final int BL_ENCODER = 10;
    public static final double BL_OFFSET = 0;

    public static final int BR_DRIVE = 6;
    public static final int BR_STEER = 7;
    public static final int BR_ENCODER = 11;
    public static final double BR_OFFSET = 0;

    // Module Geometry (inches)
    public static final double MODULE_FRONT_BACK_SPACING = 20.45;
    public static final double MODULE_LEFTRIGHT_SPACING = 25.45;

    // Module Positions (relative to robot center)
    public static final Translation2d FL_POS = new Translation2d(MODULE_FRONT_BACK_SPACING / 2.0, MODULE_LEFTRIGHT_SPACING / 2.0);
    public static final Translation2d FR_POS = new Translation2d(MODULE_FRONT_BACK_SPACING / 2.0, -MODULE_LEFTRIGHT_SPACING / 2.0);
    public static final Translation2d BL_POS = new Translation2d(-MODULE_FRONT_BACK_SPACING / 2.0, MODULE_LEFTRIGHT_SPACING / 2.0);
    public static final Translation2d BR_POS = new Translation2d(-MODULE_FRONT_BACK_SPACING / 2.0, -MODULE_LEFTRIGHT_SPACING / 2.0);

    // Kinematic Limits
    public static final double MAX_VEL = 6000.0 / SwerveDriveConstants.DRIVE_GEAR_REDUCTION / 60.0 * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();
  }

  public static class RotateToAngleConstants {
    public static final double kP = 0.005;
    public static final double kI = 0.0;
    public static final double kD = 0.0005;
    public static final double TOLERANCE_DEGREES = 2.0;
  }

  // ==================== SUBSYSTEMS ====================

  public static class IntakeConstants {
    // Roller Motor
    public static final int ROLLER_CAN_ID = 14;
    public static final double ROLLER_IN_SPEED = -1;
    public static final double ROLLER_OUT_SPEED = 1;
    public static final double ROLLER_CURRENT_LIMIT = 120.0;
    public static final double ROLLER_STATOR_CURRENT_LIMIT = 120.0;
    public static final double ROLLER_OPEN_LOOP_RAMP = 0.0;
    public static final InvertedValue ROLLER_INVERTED = InvertedValue.CounterClockwise_Positive;

    // Pivot Motor
    public static final int PIVOT_MOTOR_ID = 12;
    public static final double MANUAL_PIVOT_SPEED = 0.15;
    public static final double PIVOT_STATOR_CURRENT_LIMIT = 40.0;
    public static final boolean PIVOT_STATOR_CURRENT_LIMIT_ENABLE = true;

    // Limit Switches
    public static final int TOP_LIMIT_SWITCH_DIO = 0;
    public static final int BOTTOM_LIMIT_SWITCH_DIO = 1;
    public static final Angle TOP_LIMIT = Rotations.of(0.25);
    public static final Angle BOTTOM_LIMIT = Rotations.of(-0.1);

    // CANdle
    public static final int CANDLE_ID = 13;
  }

  public static class HopperConstants {
    // Motor Configuration
    public static final int KRAKEN_CAN_ID = 15;
    public static final InvertedValue HOPPERINVERTED = InvertedValue.Clockwise_Positive;

    // Current Limits
    public static final int SUPPLY_CURRENT_LIMIT = 80;
    public static final int STATOR_CURRENT_LIMIT = 60;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;

    // Ramp Rates
    public static final int VOLTAGE_COMPENSATION = 12;
    public static final double OPEN_LOOP_RAMP = 0.2;
    public static final double DUTY_CYCLE_OPEN_LOOP_RAMP = 0.05;

    // Manual Control
    public static final double MANUAL_SPEED = 0.5;
  }

  // ==================== VISION ====================

  public static class VisionConstants {
    public static final double FIELD_X = 16.54;
    public static final double FIELD_Y = 8.07;
    public static final double ROBOT_RADIUS = 0.762;
    public static final double[] STD_DEV_DIST = new double[] {
      0.75, 1.00, 1.3, 1.69, 2., 2.51, 2.78, 3.07, 3.54, 4.1, 4.52
    };
    public static final double[] X_STD_DEV = new double[] {
      0.002, 0.005, 0.007, 0.014, 0.029, 0.074, 0.101, 0.12, 0.151, 0.204, 0.287
    };
    public static final double[] Y_STD_DEV = new double[] {
      0.002, 0.005, 0.013, 0.020, 0.067, 0.080, 0.095, 0.160, 0.206, 0.259, 0.288
    };
    public static final double[] O_STD_DEV = new double[] {
      0.002, 0.004, 0.005, 0.011, 0.031, 0.4, 1.72, 1.89, 2.05, 2.443, 2.804
    };

    public static final CameraConfig[] cameraConfigs = new CameraConfig[]{//back top
      new CameraConfig(
        "7",
        new Transform3d(
          0.28, 0, 0,
          new Rotation3d(0, -Math.toRadians(5), 0)
        ),
        PoseStrategy.LOWEST_AMBIGUITY
      )
    };
    public static final PolynomialRegression xStdDevModel = new PolynomialRegression(
      VisionConstants.STD_DEV_DIST,VisionConstants.X_STD_DEV,2);
    public static final PolynomialRegression yStdDevModel = new PolynomialRegression(
      VisionConstants.STD_DEV_DIST,VisionConstants.Y_STD_DEV,2);
    public static final PolynomialRegression oStdDevModel = new PolynomialRegression(
      VisionConstants.STD_DEV_DIST,VisionConstants.O_STD_DEV,1);

  }

  // ==================== ALIGNMENT ====================

  public static class AlignToHubConstants {
    public static final Translation2d HUB_POSITION = new Translation2d(12.51204, 4);
  }

  public static class AlignConstants {
    public static final Translation2d BLUE_HUB_TRANS = new Translation2d(4.625, 4);
    public static final Translation2d RED_HUB_TRANS = new Translation2d(11.9, 4);
  }


  // ==================== LOGGING & DEBUG ====================

  public static class LoggingConstants {
    public static final String SWERVE_TABLE = "SwerveStats";
    public static final String SENSOR_TABLE = "Sensors";
  }

  public static class DebugConstants {
    public static final boolean MASTER_DEBUG = true;
    public static final boolean DRIVE_DEBUG = true;
    public static final boolean STEER_DEBUG = true;
    public static final boolean STATE_DEBUG = true;
  }

  public static final class ClimbConstants {
    // CAN IDs (per README: Doornob=19, Winch=21, CANdi=22)
    public static final int WINCH_MOTOR_CAN_ID = 21;
    public static final int ARM_MOTOR_CAN_ID = 19;  // Doornob
    public static final int CANDI_CAN_ID = 22;

    public static final InvertedValue ARM_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue WINCH_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

    public static final double ARM_GR = 1.0;
    public static final double WINCH_GR = 1.0;

    public static final double ARM_MAX_OUTPUT = 0.5;
    public static final double WINCH_MAX_OUTPUT = 1;

    public static final Angle ARM_REVERSE_LIMIT = Rotations.of(-0.19);
    public static final Angle ARM_FORWARD_LIMIT = Rotations.of(0);

    public static final Angle WINCH_REVERSE_LIMIT = Rotations.of(0);
    public static final Angle WINCH_FORWARD_LIMIT = Rotations.of(3);
  }
}
