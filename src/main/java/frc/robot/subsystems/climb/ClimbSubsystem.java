package frc.robot.subsystems.climb;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private StabilizingArm m_StabilizingArm;
    private Winch m_Winch;

    public ClimbSubsystem(CANBus canBusObj) {
        m_StabilizingArm = new StabilizingArm(canBusObj);
        m_Winch = new Winch(canBusObj);
    }

    private Command initiateClimb(BooleanSupplier step) {
        return m_Winch.pullUpClaw(step).andThen(Commands.waitUntil(step)).andThen(m_StabilizingArm.deployArm(step));
    }

    public Command climbUp(BooleanSupplier step) {
        return initiateClimb(step).andThen(Commands.waitUntil(step)).andThen(m_Winch.pullDownClaw(step));
    }

    public Command climbDown(BooleanSupplier step) {
        return m_Winch.pullUpClaw(step).andThen(Commands.waitUntil(step)).andThen(m_StabilizingArm.retractArm(step));
    }
}