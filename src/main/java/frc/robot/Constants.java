// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;

// Units library:
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.subsystems.Vision.FuelDetectionSubsystem;
import frc.robot.subsystems.Vision.FuelDetectionSubsystem.FuelDetectionConfig;
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
    // debug mode / pushing hella stuff to NT tables
    // Subsystem Enable/Disable
    public static final boolean SWERVE_ENABLED = true;
    public static final boolean MECH_ENABLED = true;

    public static class TowerConstants {
        public static final int KRAKEN_CAN_ID = 26;

        public enum TOWER_INTAKE {
            BALLUP,
            BALLDOWN,
            STOP
        }

        // maths
        public static final double GEAR_REDUCTION = 4.0;

        public static final double TARGET_BPS = 4.0;// frequency
        public static final double WHEEL_RADIUS = 1.0;// distance
        public static final double BALL_DIAMETER = 6.0;// distance
        public static final double TARGET_RPS = 30;// TARGET_BPS * BALL_DIAMETER / WHEEL_RADIUS;

        // Velocity control PID
        public static final double KP = 0.5;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.0;
        public static final double KV = 0.12;

        // motion magic
        public static final double MM_ACCEL = 1000;// distance
        public static final double MM_JERK = 100;// distance
        public static final double MM_MAXVELO = 100;// distance

        // Current limits
        public static final int SUPPLY_CURRENT_LIMIT = 80;
        public static final int STATOR_CURRENT_LIMIT = 60;
        public static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
        public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;

        // // Voltage and ramping
        // public static final int VOLTAGE_COMPENSATION = 12;
        // public static final double OPEN_LOOP_RAMP = 0.5;
        // public static final double DUTY_CYCLE_OPEN_LOOP_RAMP = 0.05;

        // Motor config
        public static final InvertedValue HOPPERINVERTED = InvertedValue.Clockwise_Positive;
    }

    // ==================== SHOOTER ====================
    public static class ShooterConstants {

        // ---- Flywheel ----
        public static class Flywheel {
            public static final InvertedValue F_INVERTED_VALUE = InvertedValue.Clockwise_Positive;

            public static final int UPPER_MOTOR_ID = 17;
            public static final int SECOND_MOTOR_ID = 25;

            public static final double GEAR_RATIO = 1.0;

            // Velocity control PID
            public static final double KP = 0.5;
            public static final double KI = 0.0;
            public static final double KD = 0.0;
            public static final double KS = 0.0;
            public static final double KV = 0.12;

            // Motion Magic
            public static final double MM_ACCEL = 30.0;
            public static final double MM_JERK = 150.0;
            public static final double MM_CRUISE_VELOCITY = 500.0;

            // Velocity tolerance for "at speed" check (RPS)
            public static final double VELOCITY_TOLERANCE_RPS = 2.0;
        }

        // ---- Hood ----
        public static class Hood {
            public static final int MOTOR_ID = 16;
            public static final int ENCODER_ID = 18;

            public static final double GEAR_RATIO = 244.411765;

            // Position control PID
            public static final double KP = 8.0;
            public static final double KI = 3.0;
            public static final double KD = 0.0;

            // Angle limits (rotations)
            public static final double UPPER_ANGLE_LIMIT = 0.1;
            public static final double LOWER_ANGLE_LIMIT = 0;
            public static final double INIT_ANGLE = UPPER_ANGLE_LIMIT;

            // Current limits
            public static final double STATOR_CURRENT_LIMIT = 50.0;
            public static final double SUPPLY_CURRENT_LIMIT = 40.0;
            public static final boolean CURRENT_LIMIT_ENABLE = true;

            public static final double ANGLE_TOLERANCE = 0.01;
        }
    }
    // ==================== DRIVETRAIN ====================

    public static class SwerveDriveConstants {
        // Motor Configuration
        public static final double DRIVE_PEAK_STATOR_CURRENT = 80;
        public static final double DRIVE_RAMP_RATE = 0.0;

        // Current Limits (optimized for Kraken motors)
        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 60; // Prevents brownouts
        public static final double DRIVE_STATOR_CURRENT_LIMIT = 100; // Allows burst torque for acceleration
        public static final boolean DRIVE_CURRENT_LIMIT_ENABLE = true;

        // Physical Measurements
        public static final double DRIVE_WHEEL_RADIUS = 2.0; // inches
        public static final double DRIVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(2.0 * Math.PI * DRIVE_WHEEL_RADIUS);
        public static final double DRIVE_GEAR_REDUCTION = 33.0 / 4.0; // 8.25:1

        // MotionMagic parameters for drive motors
        public static final double DRIVE_MAX_VELOCITY_RPS = 80.0; // Max velocity in rotations per second
        public static final double DRIVE_MAX_ACCELERATION = 160.0; // Max acceleration in rotations per second^2
    }

    public static class SwerveSteerConstants {
        // Motor Configuration
        public static final double STEER_PEAK_STATOR_CURRENT = 40;
        public static final double STEER_RAMP_RATE = 0;

        // Current Limits (optimized for Kraken motors - steer needs less current)
        public static final double STEER_SUPPLY_CURRENT_LIMIT = 25; // Prevents brownouts
        public static final double STEER_STATOR_CURRENT_LIMIT = 50; // Sufficient for steering
        public static final boolean STEER_CURRENT_LIMIT_ENABLE = true;

        // Physical Measurements
        public static final double STEER_GEAR_REDUCTION = 160.0 / 7.0; // ~22.86:1
        public static final double STEER_FREE_SPEED_RPM = 7530.0; // Kraken X44

        // Motion Magic
        public static final double STEER_MAX_VELOCITY = STEER_FREE_SPEED_RPM / STEER_GEAR_REDUCTION / 60.0; // ~5.49
                                                                                                            // rot/sec
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
        public static final Translation2d FL_POS = new Translation2d(MODULE_FRONT_BACK_SPACING / 2.0,
            MODULE_LEFTRIGHT_SPACING / 2.0);
        public static final Translation2d FR_POS = new Translation2d(MODULE_FRONT_BACK_SPACING / 2.0,
            -MODULE_LEFTRIGHT_SPACING / 2.0);
        public static final Translation2d BL_POS = new Translation2d(-MODULE_FRONT_BACK_SPACING / 2.0,
            MODULE_LEFTRIGHT_SPACING / 2.0);
        public static final Translation2d BR_POS = new Translation2d(-MODULE_FRONT_BACK_SPACING / 2.0,
            -MODULE_LEFTRIGHT_SPACING / 2.0);

        // Kinematic Limits
        public static final double MAX_VEL = 6000.0 / SwerveDriveConstants.DRIVE_GEAR_REDUCTION / 60.0
            * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE;
        public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();

        // Chassis Acceleration Limits (m/s^2)
        public static final double MAX_LINEAR_ACCELERATION = 3.0; // meters per second squared
        public static final double MAX_LINEAR_DECELERATION = 6; // meters per second squared
        public static final double MAX_ANGULAR_ACCELERATION = 2.0; // radians per second squared
        public static final double MAX_ANGULAR_DECELERATION = 12.0; // radians per second squared

        // Boost Mode Constants (L1 held)
        public static final double BOOST_MAX_VEL = MAX_VEL; // Use full max velocity in boost mode
        public static final double BOOST_MAX_LINEAR_ACCELERATION = 6.0; // meters per second squared
        public static final double BOOST_MAX_ANGULAR_ACCELERATION = 4.0; // radians per second squared

        // Slow Mode Constants (R1 held)
        public static final double SLOW_MODE_SPEED_LIMIT = 0.3; // 30% speed when R1 held
    }

    public static class RotateToAngleConstants {
        public static final double kP = 0.009;
        public static final double kI = 0.0;
        public static final double kD = 0.0005;
        public static final double TOLERANCE_DEGREES = 0.0;
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

        // Roller PID
        public static final double ROLLER_P = 0.1;
        public static final double ROLLER_I = 0.0;
        public static final double ROLLER_D = 0.0;
        public static final double ROLLER_V = 0.12;

        // Pivot Motor
        public static final int PIVOT_MOTOR_ID = 12;
        public static final int PIVOT_CANCODER_ID = 13;
        public static final double MANUAL_PIVOT_SPEED = 1;
        public static final double PIVOT_STATOR_CURRENT_LIMIT = 40.0;
        public static final boolean PIVOT_STATOR_CURRENT_LIMIT_ENABLE = true;

        // Pivot PID
        public static final double PIVOT_P = 50.0;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.5;
        public static final double PIVOT_F = 0.0;

        // Pivot Motion Magic
        public static final double PIVOT_CRUISE_VELOCITY = 2.0;
        public static final double PIVOT_ACCELERATION = 4.0;
        public static final double GEAR_RATIO = 25.0;

        // Pivot Positions (in rotations)
        public static final double PIVOT_OUT_POS = 0.25;
        public static final double PIVOT_IN_POS = 0.0;
        public static final double PIVOT_MID_POS = 0.125;

        // Software Limits
        public static final Angle TOP_LIMIT = Rotations.of(0.25);
        public static final Angle BOTTOM_LIMIT = Rotations.of(-0.1);
    }

    public static class HopperConstants {
        public static final int KRAKEN_CAN_ID = 15;

        public enum HOPPER_INTAKE {
            BALLIN,
            BALLOUT,
            STOP
        }

        // Velocity control PID
        public static final double KP = 0.5;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.0;
        public static final double KV = 0.12;

        // balls stuff
        public static final double TARGET_BPS = 4.0;// frequency
        public static final double GEAR_REDUCTION = 4.0; // dummy value -Tony 3.3.26
        public static final double TARGET_RPS = TARGET_BPS / 4;// divided by 4 cuz 4 vains on spinner

        // Current limits
        public static final int SUPPLY_CURRENT_LIMIT = 80;
        public static final int STATOR_CURRENT_LIMIT = 60;
        public static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
        public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;

        // Voltage and ramping
        public static final int VOLTAGE_COMPENSATION = 12;
        public static final double OPEN_LOOP_RAMP = 0.5;
        public static final double DUTY_CYCLE_OPEN_LOOP_RAMP = 0.05;

        // Motor config
        public static final InvertedValue HOPPERINVERTED = InvertedValue.CounterClockwise_Positive;
    }

    // ==================== ALIGNMENT ====================

    public static class AlignToHubConstants {
        public static final Translation2d HUB_POSITION = new Translation2d(12.51204, 4);
    }

    public static class AlignConstants {
        public static final Translation2d BLUE_HUB_TRANS = new Translation2d(4.625, 4);
        public static final Translation2d RED_HUB_TRANS = new Translation2d(11.9, 4);
        public static final double RED_WALL_X = 11.9;
        public static final double BLUE_WALL_X = 4.625;
        public static final double HUB_Y = 4;
        public static final Translation2d BLUE_AIM_TOP = new Translation2d(2.4, 6);
        public static final Translation2d BLUE_AIM_BOTTOM = new Translation2d(2.4, 2);
        public static final Translation2d RED_AIM_TOP = new Translation2d(14.3, 6);
        public static final Translation2d RED_AIM_BOTTOM = new Translation2d(14.3, 2);
    }

    // ==================== LOGGING & DEBUG ====================

    public static class LoggingConstants {
        public static final String SWERVE_TABLE = "SwerveStats";
        public static final String SENSOR_TABLE = "Sensors";
    }

    // ==================== SMASH AND SHOOT ====================
    public static class SmashAndShootConstants {
        // Hood position (rotations) - between 0.06 and 0.169
        public static final double HOOD_POSITION = 0.12;

        // Flywheel speed (RPS)
        public static final double FLYWHEEL_RPS = 50.0;

        // Tower duty cycle
        public static final double TOWER_DUTY_CYCLE = 0.7;

        // Indexer/Hopper duty cycle
        public static final double INDEXER_DUTY_CYCLE = 1.0;
    }

    public static class DebugConstants {
        public static final boolean MASTER_DEBUG = true;
        public static final boolean DRIVE_DEBUG = true;
        public static final boolean STEER_DEBUG = true;
        public static final boolean STATE_DEBUG = true;
    }

    public static final class ClimbConstants {
        // CAN IDs (Arm/Doornob=19, Winch=21, CANrange=22)
        public static final int WINCH_MOTOR_CAN_ID = 21;
        public static final int ARM_MOTOR_CAN_ID = 19; // Doornob
        public static final int CANRANGE_CAN_ID = 22;

        public static final InvertedValue ARM_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
        public static final InvertedValue WINCH_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

        public static enum CLIMB_MECH_STATE {
            HOME,
            DEPLOYED,
            FLOATING
        }

        public static final double ARM_GR = 1.0;
        public static final double WINCH_GR = 70.0;

        public static final double ARM_MAX_OUTPUT = 0.1;
        public static final double WINCH_MAX_OUTPUT = 1;

        public static final Angle ARM_ACCEPTABLE_POSITION_ERROR = Degrees.of(5);

        public static final Angle ARM_REVERSE_LIMIT = Rotations.of(-0.05);
        public static final Angle ARM_FORWARD_LIMIT = Rotations.of(0.25);
        public static final Angle WINCH_REVERSE_LIMIT = Rotations.of(-0.25);
        public static final Angle WINCH_FORWARD_LIMIT = Rotations.of(0.25);

        public static final Angle ARM_HOME_POS = Rotations.of(0.25);
        public static final Angle WINCH_HOME_POS = Rotations.of(-0.25);
        public static final Angle ARM_DEPLOYED_POS = Rotations.of(0);

        public static final Time ARM_POS_TIMEOUT = Seconds.of(5);
        public static final Time WINCH_POS_TIMEOUT = Seconds.of(5);

        public static final Angle ENCODER_OFFSET = Rotations.of(0);
        public static final Angle ENCODER_DISCONTINUITY_POINT = ((ARM_HOME_POS.plus(ARM_DEPLOYED_POS)).div(2))
            .plus(Rotations.of(0.5)); // docs for less than one rotation of travel: mean(lowerLimit,upperLimit) +
                                      // 0.5

        // Winch CANrange + torque current control
        public static final double WINCH_TORQUE_CURRENT = 10.0; // amps, placeholder to tune
        public static final Distance WINCH_HOME_DISTANCE = Millimeters.of(50); // placeholder to tune
        public static final Distance WINCH_DEPLOYED_DISTANCE = Millimeters.of(300); // placeholder to tune
        public static final Distance WINCH_DISTANCE_TOLERANCE = Millimeters.of(15); // placeholder to tune

        public static final double ARM_kP = 15;
        public static final double ARM_kI = 0.0;
        public static final double ARM_kD = 0.1;
        public static final double ARM_kG = 4.5;
        public static final double ARM_kS = 2;

        public static final String CLIMB_BASE_TABLE = "Climb";
        public static final String ARM_TABLE = CLIMB_BASE_TABLE + "/Arm";
        public static final String WINCH_TABLE = CLIMB_BASE_TABLE + "/Winch";
        public static final String STATUS_TABLE = CLIMB_BASE_TABLE + "/Status";
    }
}
