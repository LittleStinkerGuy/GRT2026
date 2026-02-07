package frc.robot.subsystems.climb;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants.CLIMB_MECH_STATE;

public class ClimbSubsystem extends SubsystemBase {
    private StabilizingArmSubsystem m_StabilizingArm;
    private WinchSubsystem m_Winch;

    private CLIMB_MECH_STATE armState = CLIMB_MECH_STATE.FLOATING;
    private CLIMB_MECH_STATE winchState = CLIMB_MECH_STATE.FLOATING;
    private CLIMB_MECH_STATE climbState = CLIMB_MECH_STATE.FLOATING;

    public ClimbSubsystem(CANBus canBusObj) {
        m_StabilizingArm = new StabilizingArmSubsystem(canBusObj);
        m_Winch = new WinchSubsystem(canBusObj);

        armState = m_StabilizingArm.getArmState();
        winchState = m_Winch.getWinchState();
    }

    public void setArmDutyCycle(double dutyCycle) {
        m_StabilizingArm.setMotorDutyCycle(dutyCycle);
    }

    public void setWinchDutyCycle(double dutyCycle) {
        m_Winch.setMotorDutyCycle(dutyCycle);
    }

    public void setArmPositionSetpoint(Angle setpoint) {
        m_StabilizingArm.setPositionSetpoint(setpoint);
    }

    public void setWinchPositionSetpoint(Angle setpoint) {
        m_Winch.setPositionSetpoint(setpoint);
    }

    public boolean isArmAtSetPosition() {
        return m_StabilizingArm.atSetPosition();
    }

    public boolean isWinchAtSetPosition() {
        return m_Winch.atSetPosition();
    }

    public boolean isArmForwardLimitActive() {
        return m_StabilizingArm.isForwardLimitActive();
    }

    public boolean isArmReverseLimitActive() {
        return m_StabilizingArm.isReverseLimitActive();
    }

    public boolean isWinchForwardLimitActive() {
        return m_Winch.isForwardLimitActive();
    }

    public boolean isWinchReverseLimitActive() {
        return m_Winch.isReverseLimitActive();
    }

    public boolean isWinchHardstopPressed() {
        return m_Winch.isHardstopPressed();
    }

    public CLIMB_MECH_STATE getClimbState() {
        armState = m_StabilizingArm.getArmState();
        winchState = m_Winch.getWinchState();

        if (armState == CLIMB_MECH_STATE.HOME && winchState == CLIMB_MECH_STATE.HOME) {
            climbState = CLIMB_MECH_STATE.HOME;
        } else if (armState == CLIMB_MECH_STATE.DEPLOYED && winchState == CLIMB_MECH_STATE.DEPLOYED) {
            climbState = CLIMB_MECH_STATE.DEPLOYED;
        } else {
            climbState = CLIMB_MECH_STATE.FLOATING;
        }
        return climbState;
    }

    public CLIMB_MECH_STATE getArmState() {
        return m_StabilizingArm.getArmState();
    }

    public CLIMB_MECH_STATE getWinchState() {
        return m_Winch.getWinchState();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("wstat", m_Winch.getWinchState().toString());
        SmartDashboard.putString("astat", m_StabilizingArm.getArmState().toString());
        SmartDashboard.putString("cstat", getClimbState().toString());
    }
}
