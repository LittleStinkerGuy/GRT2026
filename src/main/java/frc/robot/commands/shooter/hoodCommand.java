package frc.robot.commands.shooter;

import frc.robot.Constants.AlignToHubConstants;
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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SWERVE_TABLE_NAME");
    StructSubscriber<Pose2d> poseSub = table.getStructTopic("estimatedPose", Pose2d.struct).subscribe(new Pose2d());

    public hoodCommand(hood h){
        this.hd = h;
        addRequirements(hd);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        double ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignToHubConstants.HUB_POSITION));
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