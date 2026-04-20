package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.shooter.Intertable;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class HoodCommand extends Command {

    private final HoodSubsystem hd;

    public HoodCommand(HoodSubsystem h) {
        this.hd = h;
        addRequirements(hd);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double ang = SmashAndShootConstants.HOOD_POSITION;


        hd.setHoodAngle(ang);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
