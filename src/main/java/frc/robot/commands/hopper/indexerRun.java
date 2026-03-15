package frc.robot.commands.hopper;

import frc.robot.Constants.HopperConstants.HOPPER_INTAKE;
import frc.robot.subsystems.hopper.HopperSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class indexerRun extends Command {

    private HopperSubsystem hop;
    private Timer timer = new Timer();

    public indexerRun(HopperSubsystem h) {
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
        if (timer.hasElapsed(4.0)) {
            hop.setHopper(HOPPER_INTAKE.BALLIN);
        }
    }

    @Override
    public void end(boolean interrupted) {
        hop.setHopper(HOPPER_INTAKE.STOP);
    }
}
