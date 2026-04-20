package frc.robot.commands.shooter;

import com.google.flatbuffers.Table;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.Intertable;

public class RampDownFlywheel extends Command {

    private FlywheelSubsystem fly;

    public RampDownFlywheel(FlywheelSubsystem h) {
        this.fly = h;
        addRequirements(fly);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        fly.dontShoot();
    }

}
