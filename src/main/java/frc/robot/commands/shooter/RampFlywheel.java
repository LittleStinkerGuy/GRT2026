package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.shooter.Intertable;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

public class RampFlywheel extends Command {

    private final FlywheelSubsystem fly;

    public RampFlywheel(FlywheelSubsystem h) {
        this.fly = h;
        addRequirements(fly);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        fly.shoot(SmashAndShootConstants.FLYWHEEL_RPS);
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
