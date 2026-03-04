package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.Constants.AlignToHubConstants;
import frc.robot.subsystems.shooter.Intertable;

import com.google.flatbuffers.Table;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

public class rampFlywheel extends Command{
    
    private flywheel fly;
    private Intertable tableThing = new Intertable();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SWERVE_TABLE_NAME");
    StructSubscriber<Pose2d> poseSub = table.getStructTopic("estimatedPose", Pose2d.struct).subscribe(new Pose2d());
    private final NetworkTableEntry offsetEntry;

    public rampFlywheel(flywheel h){
        this.fly = h;
        addRequirements(fly);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterLearner");
        offsetEntry = table.getEntry("offset");
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        double RPS = tableThing.getRPS(poseSub.get().getTranslation().getDistance(AlignToHubConstants.HUB_POSITION));
        fly.shoot(RPS+offsetEntry.getDouble(0.0));
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
    }

}