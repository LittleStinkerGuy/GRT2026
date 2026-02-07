package frc.robot.commands.climb.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class AutoRetractArmCommand extends Command {
    private final ClimbSubsystem climbSubsystem;

    public AutoRetractArmCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setArmPositionSetpoint(ClimbConstants.ARM_HOME_POS);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isArmAtSetPosition();
    }
}
