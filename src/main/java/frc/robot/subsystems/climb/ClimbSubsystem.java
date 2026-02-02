package frc.robot.subsystems.climb;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants.CLIMB_MECH_STATE;

public class ClimbSubsystem extends SubsystemBase {
    private StabilizingArm m_StabilizingArm;
    private Winch m_Winch;

    private CLIMB_MECH_STATE armState = CLIMB_MECH_STATE.FLOATING;
    private CLIMB_MECH_STATE winchState = CLIMB_MECH_STATE.FLOATING;
    private CLIMB_MECH_STATE climbState = CLIMB_MECH_STATE.FLOATING;

    public ClimbSubsystem(CANBus canBusObj) {
        m_StabilizingArm = new StabilizingArm(canBusObj);
        m_Winch = new Winch(canBusObj);

        armState = m_StabilizingArm.getArmState();
        winchState = m_Winch.getWinchState();
    }

    public void setArmDutyCycle(double dutyCycle) {
        m_StabilizingArm.setMotorDutyCycle(dutyCycle);
    }

    public void setWinchDutyCycle(double dutyCycle) {
        m_Winch.setMotorDutyCycle(dutyCycle);
    }

    public CLIMB_MECH_STATE getClimbState() {
        if (armState == CLIMB_MECH_STATE.FLOATING && winchState == CLIMB_MECH_STATE.FLOATING) {
            climbState = CLIMB_MECH_STATE.FLOATING;
        } else if (armState == CLIMB_MECH_STATE.DEPLOYED && winchState == CLIMB_MECH_STATE.DEPLOYED) {
            climbState = CLIMB_MECH_STATE.DEPLOYED;
        } else {
            climbState = CLIMB_MECH_STATE.FLOATING;
        }
        return climbState;
    }

    private Command getClimbCommand() {
        climbState = getClimbState();

        if (climbState == CLIMB_MECH_STATE.HOME) {
            return autoClimbUp();
        } else if (climbState == CLIMB_MECH_STATE.DEPLOYED) {
            return autoClimbDown();
        } else {
            return stopMechs();
        }
    }

    public Command autoClimb() {
        return this.defer(this::getClimbCommand);
    }

    public Command autoClimbUp() {
        AtomicBoolean armInterrupted = new AtomicBoolean(false);
        Command deployArm = m_StabilizingArm.autoDeployArm().finallyDo(interrupted -> armInterrupted.set(interrupted));
        Command climbUp = deployArm
                .andThen(Commands.either(
                        Commands.none(),
                        m_Winch.autoPullDownClaw(),
                        () -> armInterrupted.get()));

        climbUp.addRequirements(this);
        return climbUp;
    }

    public Command autoClimbDown() {
        AtomicBoolean winchInterrupted = new AtomicBoolean(false);
        Command pullDownWinch = m_Winch.autoPullDownClaw().finallyDo(interrupted -> winchInterrupted.set(interrupted));
        Command climbDown = pullDownWinch
                .andThen(Commands.either(
                        Commands.none(),
                        m_StabilizingArm.autoRetractArm(),
                        () -> winchInterrupted.get()));

        climbDown.addRequirements(this);
        return climbDown;
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
    public Command semiAutoClimbUp(BooleanSupplier step) {
        Command climbUp = (waitForButtonRelease(step)
                .andThen(m_StabilizingArm.deployArm(step)
                        .raceWith(Commands.waitUntil(() -> m_StabilizingArm.getForwardLimit()))
                        .andThen(waitForNextStep(step))
                        .andThen(m_Winch.manualPullDownClaw(step))
                        .raceWith(Commands.waitUntil(() -> m_Winch.getReverseLimit()))));
        climbUp.addRequirements(this);
        return climbUp;
    }

    // winch + arm -
    // Step through the climb down sequence, stopping motors either with a button
    // press or them reaching the soft stop
    public Command semiAutoClimbDown(BooleanSupplier step) {
        Command climbDown = waitForButtonRelease(step)
                .andThen(m_Winch.manualPullUpClaw(step)
                        .raceWith(Commands.waitUntil(() -> m_Winch.getForwardLimit()))
                        .andThen(waitForNextStep(step))
                        .andThen(m_StabilizingArm.retractArm(step)
                                .raceWith(Commands.waitUntil(() -> m_StabilizingArm.getReverseLimit()))));
        climbDown.addRequirements(this);
        return climbDown;
    }

    public Command stopMechs() {
        return this.runOnce(() -> {
            setArmDutyCycle(0);
            setWinchDutyCycle(0);
        });
    }
}