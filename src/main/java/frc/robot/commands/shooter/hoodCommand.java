package frc.robot.commands.shooter;

import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.shooter.Intertable;
import frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;

public class hoodCommand extends Command {

    private final hood hd;
    private final FieldManagementSubsystem fms;
    private final Intertable tableThing = Intertable.getInstance();
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable(LoggingConstants.SWERVE_TABLE);
    private final StructSubscriber<Pose2d> poseSub;

    public hoodCommand(hood h, FieldManagementSubsystem fms) {
        this.fms = fms;
        this.hd = h;
        addRequirements(hd);

        poseSub = table.getStructTopic("estimatedPose", Pose2d.struct).subscribe(new Pose2d());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double ang;
        boolean redTeam = fms.isRedAlliance();

        if (redTeam) {
            if (poseSub.get().getX() > AlignConstants.RED_WALL_X) {
                ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.RED_HUB_TRANS));
            } else {
                if (poseSub.get().getY() > AlignConstants.HUB_Y) {
                    ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.RED_AIM_TOP));
                } else {
                    ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.RED_AIM_BOTTOM));
                }
            }

        } else {
            if (poseSub.get().getX() < AlignConstants.BLUE_WALL_X) {
                ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_HUB_TRANS));
            } else {
                if (poseSub.get().getY() > AlignConstants.HUB_Y) {
                    ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_AIM_TOP));
                } else {
                    ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_AIM_BOTTOM));
                }
            }
        }

        System.out.println("hood angle: " + ang);
        hd.setHoodAngle(ang);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
