package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.util.PIDConstants;
import frc.robot.util.ComponentStatus.MotorControlMode;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
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

        public double followerAppliedVoltage = 0.0;
        public double followerSupplyCurrentAmps = 0.0;
        public double followerTorqueCurrentAmps = 0.0;
        public double followerStatorCurrentAmps = 0.0;
        public double followerTempC = 0.0;
        public boolean followerConnected = false;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default PIDConstants getDefaultPID() {
        return PIDConstants.ZERO;
    }

    default void updatePID(double kP, double kI, double kD, double kS, double kV, double kA) {}

    default void updateMotionMagicConfig(double accelRPS2, double veloRPS, double jerkRPS3) {}

    default void setDutyCycleOut(double dutyCycle) {}

    default void setVoltageOut(double volts) {}

    default void setVelocityOut(double velocityRPS) {}

    default void stop() {}
}
