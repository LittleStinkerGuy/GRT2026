package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

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
        fly.stop();
    }

}
