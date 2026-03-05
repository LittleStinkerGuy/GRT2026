package frc.robot.commands.shooter;

import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.shooter.Intertable;
import frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Intertable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;

public class hoodCommand extends Command{

    private hood hd;
    private Intertable tableThing = new Intertable();
    private boolean redTeam = false;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SWERVE_TABLE_NAME");
    StructSubscriber<Pose2d> poseSub = table.getStructTopic("estimatedPose", Pose2d.struct).subscribe(new Pose2d());

    public hoodCommand(hood h, boolean red){
        redTeam = red;
        this.hd = h;
        addRequirements(hd);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        double ang;
        
        if(redTeam){
            ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.RED_HUB_TRANS));
        }else{
            ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_HUB_TRANS));
        }

        System.out.println("hood angle: " + ang );
        hd.setHoodAngle(ang);

    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
    @Override
    public void end(boolean interrupted){
    }
}