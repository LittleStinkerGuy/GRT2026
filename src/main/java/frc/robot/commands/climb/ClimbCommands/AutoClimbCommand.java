package frc.robot.commands.climb.ClimbCommands;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.ClimbConstants.CLIMB_MECH_STATE;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class AutoClimbCommand extends DeferredCommand {
    public AutoClimbCommand(ClimbSubsystem climbSubsystem) {
        super(selectCommand(climbSubsystem), Set.of(climbSubsystem));
    }

    private static Supplier<Command> selectCommand(ClimbSubsystem climbSubsystem) {
        return () -> {
            CLIMB_MECH_STATE state = climbSubsystem.getClimbState();
            switch (state) {
                case HOME:
                    System.out.println("up");
                    return new AutoClimbUpCommand(climbSubsystem);
                case DEPLOYED:
                    System.out.println("down");
                    return new AutoClimbDownCommand(climbSubsystem);
                default:
                    System.out.println("stop");
                    return new StopClimbMechanismsCommand(climbSubsystem);
            }
        };
    }
}
