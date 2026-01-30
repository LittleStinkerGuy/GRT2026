package frc.robot.subsystems.swerve;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AimSubsystem extends SubsystemBase{
   // Link to dimensions https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
   Transform2d fieldToHub;
   Transform2d robotToShooter;
   Pose2d fieldToRobot;


   public void AimSubsystem(Pose2d estimatedPose) {
       fieldToHub = new Transform2d(4.623,4.034, new Rotation2d());
       robotToShooter = new Transform2d(2,3,new Rotation2d(90));
       fieldToRobot = estimatedPose;


      
   }


}

