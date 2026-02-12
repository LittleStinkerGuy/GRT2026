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

    public void setArmDutyCycle(double dutyCycle) {
        m_StabilizingArm.setMotorDutyCycle(dutyCycle);
    }

    public void setWinchDutyCycle(double dutyCycle) {
        m_Winch.setMotorDutyCycle(dutyCycle);
    }

    // block command execution until the booleanSupplier, usually a button, is
    // released
    private Command waitForButtonRelease(BooleanSupplier step) {
        return Commands.waitUntil(() -> !step.getAsBoolean());
    }

    // block command execution until the booleanSupplier, usually a button, toggles
    // back and forth
    private Command waitForNextStep(BooleanSupplier step) {
        return waitForButtonRelease(step).andThen(Commands.waitUntil(step)).andThen(waitForButtonRelease(step));
    }

    // arm + winch -
    // Step through the climb up sequence, stopping motors either with a button
    // press or them reaching the soft stop
    // public Command climbUp(BooleanSupplier step) {
    //     Command climbUp = (waitForButtonRelease(step)
    //             .andThen(m_StabilizingArm.deployArm(step)
    //                     .raceWith(Commands.waitUntil(() -> m_StabilizingArm.getForwardLimit().orElseGet(() -> false)))
    //                     .andThen(waitForNextStep(step))
    //                     .andThen(m_Winch.pullDownClaw(step))
    //                     .raceWith(Commands
    //                             .waitUntil(() -> m_Winch.getReverseLimit().orElseGet(() -> false)))));
    //     climbUp.addRequirements(this);
    //     return climbUp;
    // }

    // // winch + arm -
    // // Step through the climb down sequence, stopping motors either with a button
    // // press or them reaching the soft stop
    // public Command climbDown(BooleanSupplier step) {
    //     Command climbDown = waitForButtonRelease(step)
    //             .andThen(m_Winch.pullUpClaw(step)
    //                     .raceWith(Commands.waitUntil(() -> m_Winch.getForwardLimit().orElseGet(() -> false)))
    //                     .andThen(waitForNextStep(step))
    //                     .andThen(m_StabilizingArm.retractArm(step)
    //                             .raceWith(Commands
    //                                     .waitUntil(() -> m_StabilizingArm.getReverseLimit().orElseGet(() -> false)))));
    //     climbDown.addRequirements(this);
    //     return climbDown;
    // }
}
