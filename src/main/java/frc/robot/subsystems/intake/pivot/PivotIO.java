package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.util.ComponentStatus.EncoderHealth;
import frc.robot.util.ComponentStatus.MotorControlMode;
import frc.robot.util.PIDConstants;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double positionRot = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempC = 0.0;
        public boolean tempFault = false;
        public boolean motorConnected = false;

        public double encoderAbsolutePositionRot = 0.0;
        public EncoderHealth encoderHealth = EncoderHealth.Unknown;
        public boolean encoderConnected = false;

        public MotorControlMode controlMode = MotorControlMode.Disabled;
        public double appliedDutyCycle = 0.0;
        public double closedLoopSetpoint = 0.0;
        public double closedLoopOutput = 0.0;
    }

    default void updateInputs(PivotIOInputs inputs) {}

    default PIDConstants getDefaultPID() {
        return PIDConstants.ZERO;
    }

    default void updatePID(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {}

    default void setDutyCycleOut(double dutyCycle) {}

    default void setVoltageOut(double volts) {}

    default void setPositionOut(double positionRot) {}

    default void stop() {}
}
