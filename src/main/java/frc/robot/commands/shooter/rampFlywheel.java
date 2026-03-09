package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.subsystems.shooter.Intertable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

public class rampFlywheel extends Command {

    private final FieldManagementSubsystem fms;
    private final flywheel fly;
    private final Intertable tableThing = Intertable.getInstance();
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable(LoggingConstants.SWERVE_TABLE);
    private final StructSubscriber<Pose2d> poseSub;
    private final NetworkTableEntry offsetEntry;

    public rampFlywheel(flywheel h, FieldManagementSubsystem fms) {
        this.fms = fms;
        this.fly = h;
        addRequirements(fly);

        poseSub = table.getStructTopic("estimatedPose", Pose2d.struct).subscribe(new Pose2d());
        NetworkTable learnerTable = NetworkTableInstance.getDefault().getTable("ShooterLearner");
        offsetEntry = learnerTable.getEntry("offset");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double RPS = 0;
        boolean redTeam = fms.isRedAlliance();

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
