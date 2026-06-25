package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.util.PIDConstants;
import frc.robot.util.ComponentStatus.MotorControlMode;

public interface RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public double positionRot = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempC = 0.0;
        public boolean tempFault = false;
        public boolean connected = false;

        public MotorControlMode controlMode = MotorControlMode.Disabled;
        public double appliedDutyCycle = 0.0;
        public double closedLoopSetpoint = 0.0;
        public double closedLoopOutput = 0.0;
    }

    default void updateInputs(RollerIOInputs inputs) {}

    default PIDConstants getDefaultPID() {
        return PIDConstants.ZERO;
    }

    default void updatePID(double kP, double kI, double kD, double kS, double kV, double kA) {}

    default void setDutyCycleOut(double dutyCycle) {}

    default void setVoltageOut(double voltsOut) {}

    default void setVelocityOut(double velocityOutRPS) {}

    default void stop() {}
}
