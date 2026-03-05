package frc.robot.commands.allign;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.AlignConstants;

public class AimBot extends Command{

    private SwerveSubsystem swerve;
    private FieldManagementSubsystem field;
    private boolean redTeam = false;
    
    public AimBot(SwerveSubsystem swerveSubsystem, FieldManagementSubsystem fms, boolean red){
        swerve = swerveSubsystem;
        field = fms;
        redTeam = red;
    }

     @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        
        

    }

}
