package frc.robot.commands.shooter;

import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.shooter.Intertable;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;

public class hoodCommand extends Command {

    private final hood hd;
    private final SwerveSubsystem swerve;
    private final FieldManagementSubsystem fms;
    private final Intertable tableThing = Intertable.getInstance();

    public hoodCommand(hood h, SwerveSubsystem swerve, FieldManagementSubsystem fms) {
        this.hd = h;
        this.swerve = swerve;
        this.fms = fms;
        addRequirements(hd);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double ang;
        boolean redTeam = fms.isRedAlliance();
        Pose2d pose = swerve.getRobotPosition();

        if (redTeam) {
            if (pose.getX() > AlignConstants.RED_WALL_X) {
                ang = tableThing.getAngle(pose.getTranslation().getDistance(AlignConstants.RED_HUB_TRANS));
            } else {
                if (pose.getY() > AlignConstants.HUB_Y) {
                    ang = tableThing.getAngle(pose.getTranslation().getDistance(AlignConstants.RED_AIM_TOP));
                } else {
                    ang = tableThing.getAngle(pose.getTranslation().getDistance(AlignConstants.RED_AIM_BOTTOM));
                }
            }

        } else {
            if (pose.getX() < AlignConstants.BLUE_WALL_X) {
                ang = tableThing.getAngle(pose.getTranslation().getDistance(AlignConstants.BLUE_HUB_TRANS));
            } else {
                if (pose.getY() > AlignConstants.HUB_Y) {
                    ang = tableThing.getAngle(pose.getTranslation().getDistance(AlignConstants.BLUE_AIM_TOP));
                } else {
                    ang = tableThing.getAngle(pose.getTranslation().getDistance(AlignConstants.BLUE_AIM_BOTTOM));
                }
            }
        }

        hd.setHoodAngle(ang);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
