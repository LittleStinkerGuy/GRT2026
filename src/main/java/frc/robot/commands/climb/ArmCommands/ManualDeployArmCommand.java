package frc.robot.commands.climb.ArmCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ManualDeployArmCommand extends Command {
    private final ClimbSubsystem m_climbSubsystem;
    private final BooleanSupplier stopSupplier;

    public ManualDeployArmCommand(ClimbSubsystem climbSubsystem, BooleanSupplier stopSupplier) {
        this.m_climbSubsystem = climbSubsystem;
        this.stopSupplier = stopSupplier;
        addRequirements(m_climbSubsystem);
    }

    @Override
    public void initialize() {
        m_climbSubsystem.setArmDutyCycle(-1);
    }

    @Override
    public void end(boolean interrupted) {
        m_climbSubsystem.setArmDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return stopSupplier.getAsBoolean();
    }
}
