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

    public hoodCommand(hood h) {
        this.hd = h;
        addRequirements(hd);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double ang = 0.15;


        hd.setHoodAngle(ang);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
