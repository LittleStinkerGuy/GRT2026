package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants.HopperIntake;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;

public class RunIndexer extends Command {

    private HopperSubsystem hop;
    private Timer timer = new Timer();

    public RunIndexer(HopperSubsystem h) {
        hop = h;
        addRequirements(hop);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(1)) {
            // hop.setHopper(HopperIntake.BALL_IN);
            hop.setManualControl(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // hop.setHopper(HopperIntake.STOP);
        hop.setManualControl(0);

    }
}
