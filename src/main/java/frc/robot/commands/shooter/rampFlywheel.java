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

    private final flywheel fly;

    public rampFlywheel(flywheel h) {
        this.fly = h;
        addRequirements(fly);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double RPS = 0;
        fly.shoot(RPS);
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
