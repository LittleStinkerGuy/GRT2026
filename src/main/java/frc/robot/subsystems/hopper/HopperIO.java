package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MotorControlMode;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        public Angle position;
        public AngularVelocity velocity;
        public AngularAcceleration acceleration;
        public Voltage appliedVoltage;
        public Current supplyCurrent;
        public Current torqueCurrent;
        public Current statorCurrent;
        public Temperature temp;
        public boolean tempFault;
        public boolean connected;

        public MotorControlMode controlMode;
        public double appliedDutyCycle;
        public double closedLoopReference;
        public double closedLoopOutput;
    }


    default void updateInputs(HopperIOInputs inputs) {}

    default void updatePID(double kP, double kI, double kD, double kS, double kV, double kA) {}

    default void updateMotionMagicConfig(AngularAcceleration accel, AngularVelocity velo, Velocity<AngularAccelerationUnit> jerk) {}

    default void setDutyCycleOut(double dutyCycle) {}

    default void setVoltageOut(Voltage voltsOut) {}

    default void setVelocityOut(AngularVelocity velocityOut) {}

    default void stop() {}
}
