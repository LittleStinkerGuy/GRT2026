// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AlignUtil;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveDriveConstants {

    // Motor Constants (DRIVE)
    public static final double DRIVE_PEAK_CURRENT = 80; // Maximum current limit for the motor in amps
    public static final double DRIVE_RAMP_RATE = 0; // Time in seconds for the motor to go from neutral to full throttle

    // Physical Measurements (DRIVE)
    public static final double DRIVE_WHEEL_RADIUS = 2.0; // Wheel radius in inches
    public static final double DRIVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(2.0 * Math.PI * DRIVE_WHEEL_RADIUS); // Circumference of the drive wheel in meters
    public static final double DRIVE_GEAR_REDUCTION = 33.0 / 4.0; // Gear reduction ratio for Drive (8.25)


  }
  public static class SwerveSteerConstants{
      // Motor Constants (STEER)
      public static final double STEER_PEAK_CURRENT = 80; // Maximum current limit for the motor in amps
      public static final double STEER_RAMP_RATE = 0; // Time in seconds for the motor to go from neutral to full throttle

      // Physical Measurements (STEER)
      public static final double STEER_GEAR_REDUCTION = 160.0 / 7.0; // Gear reduction ratio for steer (22.857142...)


  }

  public static class SwerveConstants{

    // Swerve Drive PID values (Velocity Control)
    public static final double[] DRIVE_P = new double[] {9.5, 9.5, 9.5, 9.5};
    public static final double[] DRIVE_I = new double[] {0, 0, 0, 0};
    public static final double[] DRIVE_D = new double[] {0.1, 0.1, 0.1, 0.1};
    public static final double[] DRIVE_S = new double[] {0.5, 0.5, 0.5, 0.5}; // Static friction compensation
    public static final double[] DRIVE_V = new double[] {0.12, 0.12, 0.12, 0.12}; // Velocity feedforward

    // Swerve Steer PID values (Position Control)
    public static final double[] STEER_P = new double[] {100, 100, 100, 100};
    public static final double[] STEER_I = new double[] {0, 0, 0, 0};
    public static final double[] STEER_D = new double[] {0.1, 0.1, 0.1, 0.1};
    public static final double[] STEER_FF = new double[] {0, 0, 0, 0}; 

    // Front Left Module
    public static final int    FL_DRIVE   = 0;
    public static final int    FL_STEER   = 4;
    public static final int    FL_ENCODER = 11;
    public static final double FL_OFFSET  = 0;

    // Front Right Module
    public static final int    FR_DRIVE   = 1;
    public static final int    FR_STEER   = 5;
    public static final int    FR_ENCODER = 10;
    public static final double FR_OFFSET  = 0;

    // Back Left Module
    public static final int    BL_DRIVE   = 2;
    public static final int    BL_STEER   = 6;
    public static final int    BL_ENCODER = 8;
    public static final double BL_OFFSET  = 0;

    // Back Right Module
    public static final int    BR_DRIVE   = 3;
    public static final int    BR_STEER   = 7;
    public static final int    BR_ENCODER = 9;
    public static final double BR_OFFSET  = 0;

    // Module distance from center (in meters)
    // Square configuration: distance between adjacent modules (FL to FR, or FL to BL) is 25.450 inches
    public static final double MODULE_SPACING = 25.450; // Distance between adjacent modules in inches
    public static final double MODULE_DIST_FROM_CENTER = Units.inchesToMeters(MODULE_SPACING / 2.0); // Half the spacing = distance from center

    // Module positions relative to robot center (square configuration)
    public static final Translation2d FL_POS = new Translation2d( MODULE_DIST_FROM_CENTER,  MODULE_DIST_FROM_CENTER);
    public static final Translation2d FR_POS = new Translation2d( MODULE_DIST_FROM_CENTER, -MODULE_DIST_FROM_CENTER);
    public static final Translation2d BL_POS = new Translation2d(-MODULE_DIST_FROM_CENTER,  MODULE_DIST_FROM_CENTER);
    public static final Translation2d BR_POS = new Translation2d(-MODULE_DIST_FROM_CENTER, -MODULE_DIST_FROM_CENTER);

    // Maximum velocity calculation: Kraken max RPM / gear ratio / 60 (convert to per second) * wheel circumference
    public static final double MAX_VEL = 6000.0 / SwerveDriveConstants.DRIVE_GEAR_REDUCTION / 60.0 * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();
    


  }

  

  public static class LoggingConstants{
    public static final String SWERVE_TABLE = "SwerveStats";
    public static final String SENSOR_TABLE = "Sensors";
  }

  public static class DebugConstants{
    public static final boolean MASTER_DEBUG = true;
    public static final boolean DRIVE_DEBUG = false;
    public static final boolean STEER_DEBUG = true;
    public static final boolean STATE_DEBUG = true;
  }

  public static class AlignConstants{
    public static String reefName = "reefAlignPath";
    public static String sourceName = "sourceAlignPath";
    public static String A_alignName = "A align";
    public static String B_alignName = "B align";
    public static String C_alignName = "C align";
    public static String D_alignName = "D align";
    public static String E_alignName = "E align";
    public static String F_alignName = "F align";
    public static String G_alignName = "G align";
    public static String H_alignName = "H align";
    public static String I_alignName = "I align";
    public static String J_alignName = "J align";
    public static String K_alignName = "K align";
    public static String L_alignName = "L align";
  
    public final static List<Pose2d> blueLeftReefPoseList = List.of(
      AlignUtil.getAlignPath(A_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(C_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(E_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(G_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(I_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(K_alignName).getStartingHolonomicPose().get()
    );
    
    public final static List<Pose2d> blueRightReefPoseList = List.of(
      AlignUtil.getAlignPath(B_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(D_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(F_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(H_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(J_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(L_alignName).getStartingHolonomicPose().get()
    ); 
    
    public final static List<Pose2d> redLeftReefPoseList = List.of(
      AlignUtil.getAlignPath(A_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(C_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(E_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(G_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(I_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(K_alignName).flipPath().getStartingHolonomicPose().get()
    );
  
    public final static List<Pose2d> redRightReefPoseList = List.of(
      AlignUtil.getAlignPath(B_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(D_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(F_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(H_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(J_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(L_alignName).flipPath().getStartingHolonomicPose().get()
    ); 
    
    //put the left on top of right
    public final static List<Pose2d> allReefPoseList = List.of(
      AlignUtil.getAlignPath(B_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(C_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(D_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(E_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(F_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(G_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(H_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(I_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(J_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(K_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(L_alignName).getStartingHolonomicPose().get()
    );

      //put the left on top of right
    public final static List<String> reefPathList = List.of(
        A_alignName,
        B_alignName,
        C_alignName,
        D_alignName,
        E_alignName,
        F_alignName,
        G_alignName,
        H_alignName,
        I_alignName,
        J_alignName,
        K_alignName,
        L_alignName
    );

    public final static List<ChassisSpeeds> reefdirectionList = List.of(
      new ChassisSpeeds(-.5, 0, 0),
      new ChassisSpeeds(-.25, -.25, 0),
      new ChassisSpeeds(.25, -.25, 0),
      new ChassisSpeeds(.5, 0, 0),
      new ChassisSpeeds(.25, .25, 0),
      new ChassisSpeeds(-.25, .25, 0)
    );

    public static String LS_1alignName  = "LS align 1";
    public static String LS_2alignName  = "LS align 2";
    public static String LS_3alignName  = "LS align 3";
    public static String RS_1alignName  = "RS align 1";
    public static String RS_2alignName  = "RS align 2";
    public static String RS_3alignName  = "RS align 3";


    public static final List<Pose2d> blueSourcePoses = List.of(
      // AlignUtil.getAlignPath(LS_1alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(LS_2alignName).getStartingHolonomicPose().get(),
      // AlignUtil.getAlignPath(LS_3alignName).getStartingHolonomicPose().get(),
      // AlignUtil.getAlignPath(RS_1alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(RS_2alignName).getStartingHolonomicPose().get()
      // AlignUtil.getAlignPath(RS_3alignName).getStartingHolonomicPose().get()
    );

    public static final List<Pose2d> redSourcePoses = List.of(
      // AlignUtil.getAlignPath(LS_1alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(LS_2alignName).flipPath().getStartingHolonomicPose().get(),
     // AlignUtil.getAlignPath(LS_3alignName).flipPath().getStartingHolonomicPose().get(),
     // AlignUtil.getAlignPath(RS_1alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(RS_2alignName).flipPath().getStartingHolonomicPose().get()
      //AlignUtil.getAlignPath(RS_3alignName).flipPath().getStartingHolonomicPose().get()
    );

    public static final List<String> sourcePathList = List.of(
     // LS_1alignName,
      LS_2alignName,
     // LS_3alignName,
     // RS_1alignName,
      RS_2alignName
     // RS_3alignName
    );

    public static String algae_1 = "Algae Align 1";
    public static String algae_2 = "Algae Align 2";
    public static String algae_3 = "Algae Align 3";
    public static String algae_4 = "Algae Align 4";
    public static String algae_5 = "Algae Align 5";
    public static String algae_6 = "Algae Align 6";

    public static List<String> algaeAlignNames = List.of(
      algae_1,
      algae_2,
      algae_3,
      algae_4,
      algae_5,
      algae_6
    );

    public static List<Pose2d> blueAlgaeAlignPoses = List.of(
      AlignUtil.getAlignPath(algae_1).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_2).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_3).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_4).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_5).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_6).getStartingHolonomicPose().get()
    );

    public static List<Pose2d> redAlgaeAlignPoses = List.of(
      AlignUtil.getAlignPath(algae_1).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_2).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_3).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_4).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_5).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_6).flipPath().getStartingHolonomicPose().get()
    );
    


    public static double distanceTolerance = .47; 
  }

public static final class ClimbConstants {
    public static final int WINCH_MOTOR_CAN_ID = 0;
    public static final int ARM_MOTOR_CAN_ID = 1;
    public static final int TOF_CAN_ID = 2;

    public static final InvertedValue ARM_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue WINCH_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final double ARM_GR = 1.0;
    public static final double WINCH_GR = 1.0;
  }
}
