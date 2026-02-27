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
    private final SwerveSubsystem swerveSubsystem;
    private final FieldManagementSubsystem fmsSubsystem;
    // Shooter offset relative to robot center (x: forward/back, y: left/right in meters)
    private static final Translation2d SHOOTER_OFFSET = new Translation2d(-0.08, 0.073);
    private static final Rotation2d SHOOTER_ANGLE = new Rotation2d(-Math.PI);

    // Link to dimensions https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public AimToHubCommand (SwerveSubsystem swerveSubsystem, FieldManagementSubsystem fmsSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.fmsSubsystem = fmsSubsystem;
        addRequirements(swerveSubsystem);
    }

    public double calculateTargetAngle() {
        Pose2d robotPose = swerveSubsystem.getRobotPosition();

        // Get the hub position based on alliance
        Translation2d hubTrans;
        if(fmsSubsystem.isRedAlliance()){
            hubTrans = AlignConstants.RED_HUB_TRANS;
        } else {
            hubTrans = AlignConstants.BLUE_HUB_TRANS;
        }
       Translation2d shooterPosition = robotPose.getTranslation().plus(SHOOTER_OFFSET);
       Translation2d shooterToHub = hubTrans.minus(shooterPosition);   
       Rotation2d targetAngle = shooterToHub.getAngle().minus(SHOOTER_ANGLE);
       
       return targetAngle.getDegrees() - robotPose.getRotation().getDegrees();
   }

    public Command createAimCommand(BooleanSupplier cancelCondition){
        double targetAngle = calculateTargetAngle();
        return new RotateByAngleCommand(swerveSubsystem, targetAngle, cancelCondition);
    }

    public double getDistanceToHub(){
        Translation2d robotPosition = swerveSubsystem.getRobotPosition().getTranslation();

        // Get the hub position based on alliance
        Translation2d hubTrans;
        if(fmsSubsystem.isRedAlliance()){
            hubTrans = AlignConstants.RED_HUB_TRANS;
        } else {
            hubTrans = AlignConstants.BLUE_HUB_TRANS;
        }

        return robotPosition.getDistance(hubTrans);
    }
}

