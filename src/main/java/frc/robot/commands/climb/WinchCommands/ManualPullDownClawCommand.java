package frc.robot.commands.climb.WinchCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import java.util.function.BooleanSupplier;

public class ManualPullDownClawCommand extends Command {
    private final ClimbSubsystem m_climbSubsystem;
    private final BooleanSupplier stopSupplier;

    public ManualPullDownClawCommand(ClimbSubsystem climbSubsystem, BooleanSupplier stopSupplier) {
        this.m_climbSubsystem = climbSubsystem;
        this.stopSupplier = stopSupplier;
        addRequirements(m_climbSubsystem);
    }

    @Override
    public void initialize() {
        m_climbSubsystem.setWinchDutyCycle(-1);
    }

    @Override
    public void end(boolean interrupted) {
        m_climbSubsystem.setWinchDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return stopSupplier.getAsBoolean();
    }
}
