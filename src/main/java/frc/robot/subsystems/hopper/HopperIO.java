package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
        public Angle position = Rotations.of(0);
        public AngularVelocity velocity = RotationsPerSecond.of(0);
        public AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0);
        public Voltage appliedVoltage = Volts.of(0);
        public Current supplyCurrent = Amps.of(0);
        public Current torqueCurrent = Amps.of(0);
        public Current statorCurrent = Amps.of(0);
        public Temperature temp = Celsius.of(0);
        public boolean tempFault = false;
        public boolean connected = false;

        public MotorControlMode controlMode = MotorControlMode.Disabled;
        public double appliedDutyCycle = 0.0;
        public double closedLoopReference = 0.0;
        public double closedLoopOutput = 0.0;
    }


    default void updateInputs(HopperIOInputs inputs) {}

    default void updatePID(double kP, double kI, double kD, double kS, double kV, double kA) {}

    default void updateMotionMagicConfig(AngularAcceleration accel, AngularVelocity velo, Velocity<AngularAccelerationUnit> jerk) {}

    default void setDutyCycleOut(double dutyCycle) {}

    default void setVoltageOut(Voltage voltsOut) {}

    default void setVelocityOut(AngularVelocity velocityOut) {}

    default void stop() {}
}
