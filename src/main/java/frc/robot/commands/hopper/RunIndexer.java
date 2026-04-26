package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;

public class RunIndexer extends Command {

    private HopperSubsystem hopper;
    private Timer timer = new Timer();

    public RunIndexer(HopperSubsystem h) {
        hopper = h;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(1)) {
            hopper.setDutyCycle(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stop();

    }
}
