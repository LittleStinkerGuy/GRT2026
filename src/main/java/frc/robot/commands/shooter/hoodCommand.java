package frc.robot.commands.shooter;

import frc.robot.Constants.AlignToHubConstants;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.table;

public class hoodCommand extends Command{

    private hood hd;
    private SwerveSubsystem s;
    
    public hoodCommand(hood h, SwerveSubsystem sw){
        hd = h;
        s=sw;
        addRequirements(hd, s);
    }

    @Override
    public void execute() {
        double ang = table.getAngle(s.getRobotPosition().getTranslation().getDistance(AlignToHubConstants.HUB_POSITION));
        hd.setHoodAngle(ang);
    }

    @Override
    public void end(boolean interrupted){
    }
}