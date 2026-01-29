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

    public void setArmSpeed(double speed) {
        m_StabilizingArm.setMotorSpeed(speed);
    }

    public void setWinchSpeed(double speed) {
        m_Winch.setMotorSpeed(speed);
    }

    private Command waitForButtonRelease(BooleanSupplier step) {
        return Commands.waitUntil(() -> !step.getAsBoolean());
    }

    private Command waitForNextStep(BooleanSupplier step) {
        return waitForButtonRelease(step).andThen(Commands.waitUntil(step)).andThen(waitForButtonRelease(step))
                .andThen(this.runOnce(() -> {
                    System.out.println("step");
                }));
    }

    private Command initiateClimb(BooleanSupplier step) {
        return m_Winch.pullUpClaw(step).andThen(waitForNextStep(step)).andThen(m_StabilizingArm.deployArm(step));
    }

    public Command climbUp(BooleanSupplier step) {
        return waitForButtonRelease(step)
                .andThen(initiateClimb(step).andThen(waitForNextStep(step)).andThen(m_Winch.pullDownClaw(step)));
    }

    public Command climbDown(BooleanSupplier step) {
        return waitForButtonRelease(step).andThen(
                m_Winch.pullUpClaw(step).andThen(waitForNextStep(step)).andThen(m_StabilizingArm.retractArm(step)));
    }
}