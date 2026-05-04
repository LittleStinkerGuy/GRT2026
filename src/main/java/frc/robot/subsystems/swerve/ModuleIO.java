package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.ComponentStatus.EncoderHealth;
import frc.robot.util.ComponentStatus.MotorControlMode;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public Angle drivePosition = Rotations.of(0);
        public AngularVelocity driveVelocity = RotationsPerSecond.of(0);
        public AngularAcceleration driveAcceleration = RotationsPerSecondPerSecond.of(0);
        public Voltage driveAppliedVoltage = Volts.of(0);
        public Current driveSupplyCurrent = Amps.of(0);
        public Current driveTorqueCurrent = Amps.of(0);
        public Current driveStatorCurrent = Amps.of(0);
        public Temperature driveTemp = Celsius.of(0);
        public boolean driveTempFault = false;
        public boolean driveMotorConnected = false;

        public MotorControlMode driveControlMode = MotorControlMode.Disabled;
        public double driveAppliedDutyCycle = 0.0;
        public double driveClosedLoopSetpoint = 0.0;
        public double driveClosedLoopOutput = 0.0;

        public Angle steerPosition = Rotations.of(0);
        public AngularVelocity steerVelocity = RotationsPerSecond.of(0);
        public AngularAcceleration steerAcceleration = RotationsPerSecondPerSecond.of(0);
        public Voltage steerAppliedVoltage = Volts.of(0);
        public Current steerSupplyCurrent = Amps.of(0);
        public Current steerTorqueCurrent = Amps.of(0);
        public Current steerStatorCurrent = Amps.of(0);
        public Temperature steerTemp = Celsius.of(0);
        public boolean steerTempFault = false;
        public boolean steerMotorConnected = false;

        public MotorControlMode steerControlMode = MotorControlMode.Disabled;
        public double steerAppliedDutyCycle = 0.0;
        public double steerClosedLoopSetpoint = 0.0;
        public double steerClosedLoopOutput = 0.0;

        public Angle encoderAbsolutePosition = Rotations.of(0.0);
        public EncoderHealth encoderHealth = EncoderHealth.Unknown;
        public boolean encoderConnected = false;

        public double[] odometryDrivePositionsRads = new double[] {};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVelocity(AngularVelocity velocity, Voltage feedForward) {}

    public default void setDriveVelocity(AngularVelocity velocity) {}

    public default void setDriveVoltage(Voltage voltage) {}

    public default void setSteerPosition(Angle rotation) {}

    public default void setSteerVoltage(Voltage voltage) {}

    public default void stopSteer() {}

    public default void stopDrive() {}

    public default void setDrivePID(double kP, double kI, double kD, double kS, double kV) {}

    public default void setSteerPID(double kP, double kI, double kD, double kS, double kV) {}

}
