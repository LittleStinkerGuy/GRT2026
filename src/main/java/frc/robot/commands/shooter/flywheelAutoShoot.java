package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.AlignToHubConstants;
import frc.robot.subsystems.shooter.Intertable;

import com.google.flatbuffers.Table;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

public class flywheelAutoShoot extends Command {

    private boolean redTeam = false;
    private flywheel fly;
    private Intertable tableThing = Intertable.getInstance();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SWERVE_TABLE_NAME");
    StructSubscriber<Pose2d> poseSub = table.getStructTopic("estimatedPose", Pose2d.struct).subscribe(new Pose2d());
    private final NetworkTableEntry offsetEntry;

    public flywheelAutoShoot(flywheel h, boolean red) {
        redTeam = red;
        this.fly = h;
        addRequirements(fly);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterLearner");
        offsetEntry = table.getEntry("offset");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double RPS = 0;

        if (redTeam) {
            if (poseSub.get().getX() > AlignConstants.RED_WALL_X) {
                RPS = tableThing.getRPS(poseSub.get().getTranslation().getDistance(AlignConstants.RED_HUB_TRANS));
            } else {
                if (poseSub.get().getY() > AlignConstants.HUB_Y) {
                    RPS = tableThing.getRPS(poseSub.get().getTranslation().getDistance(AlignConstants.RED_AIM_TOP));
                } else {
                    RPS = tableThing.getRPS(poseSub.get().getTranslation().getDistance(AlignConstants.RED_AIM_BOTTOM));
                }
            }

        } else {
            if (poseSub.get().getX() < AlignConstants.BLUE_WALL_X) {
                RPS = tableThing.getRPS(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_HUB_TRANS));
            } else {
                if (poseSub.get().getY() > AlignConstants.HUB_Y) {
                    RPS = tableThing.getRPS(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_AIM_TOP));
                } else {
                    RPS = tableThing.getRPS(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_AIM_BOTTOM));
                }
            }
        }

        fly.shoot(RPS + offsetEntry.getDouble(0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        fly.dontShoot();
    }

}
