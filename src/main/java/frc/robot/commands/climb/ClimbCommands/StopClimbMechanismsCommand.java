package frc.robot.commands.climb.ClimbCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class StopClimbMechanismsCommand extends InstantCommand {
    public StopClimbMechanismsCommand(ClimbSubsystem climbSubsystem) {
        super(() -> {
            System.out.println("stop");
            climbSubsystem.setArmDutyCycle(0);
            climbSubsystem.setWinchDutyCycle(0);
        }, climbSubsystem);
    }
}
