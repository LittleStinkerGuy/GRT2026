package frc.robot.commands.allign;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.AlignConstants;

public class AimToHubCommand extends Command{
    static SwerveSubsystem swerveSubsystem;
    static FieldManagementSubsystem fmsSubsystem;
    static Translation2d hubTrans;
    static Transform2d shooterOffset = new Transform2d(-0.08, 0.073, new Rotation2d(-Math.PI/2));

    // Link to dimensions https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public AimToHubCommand (SwerveSubsystem swerveSubsystem, FieldManagementSubsystem fmsSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.fmsSubsystem = fmsSubsystem;
    }
          
    public static double AimMath() {
        Pose2d estimatedPose = swerveSubsystem.getRobotPosition();

        if(fmsSubsystem.isRedAlliance()){
            hubTrans = AlignConstants.RED_HUB_TRANS;
        }
        else{
            hubTrans = AlignConstants.BLUE_HUB_TRANS;
        }
            
        Translation2d shooterPosition = estimatedPose.getTranslation().plus(shooterOffset.getTranslation());
        Translation2d shooterToHub = hubTrans.minus(shooterPosition);    
        Rotation2d targetAngle = shooterToHub.getAngle().minus(shooterOffset.getRotation());

        return targetAngle.getDegrees();
    }

    public static Command Aim (BooleanSupplier cancelCondition){
        double targetAngle = AimMath();
        return new RotateToAngleCommand(swerveSubsystem, targetAngle, cancelCondition);
    }

    public static double distToHub(){
        Translation2d estimatedTrans = swerveSubsystem.getRobotPosition().getTranslation();
        return estimatedTrans.getDistance(hubTrans);
    }
}

