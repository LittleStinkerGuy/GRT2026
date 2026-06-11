package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveSteerConstants.STEER_ACCELERATION;
import static frc.robot.Constants.SwerveSteerConstants.STEER_CRUISE_VELOCITY;
import static frc.robot.Constants.SwerveSteerConstants.STEER_CURRENT_LIMIT_ENABLE;
import static frc.robot.Constants.SwerveSteerConstants.STEER_GEAR_REDUCTION;
import static frc.robot.Constants.SwerveSteerConstants.STEER_PEAK_STATOR_CURRENT;
import static frc.robot.Constants.SwerveSteerConstants.STEER_RAMP_RATE;
import static frc.robot.Constants.SwerveSteerConstants.STEER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.SwerveSteerConstants.STEER_SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SteerMotor extends SubsystemBase {
    private double gurtMotorPos = 0.0;
    private double targetPos = 0.0;
    private int canID;
    private int motorID;
    private TalonFX motor;
    private CANcoder cancoder;
    private final boolean enableEncoder = true;
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    private PositionTorqueCurrentFOC posTorqueCurrentFOCRequest = new PositionTorqueCurrentFOC(0)
        .withSlot(0)
        .withUpdateFreqHz(100.0);

    private void configureMotor() {
        // Set peak current for torque limiting for stall prevention
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = STEER_PEAK_STATOR_CURRENT;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -STEER_PEAK_STATOR_CURRENT;

        // Current limits (optimized for swerve steer)
        motorConfig.CurrentLimits.SupplyCurrentLimit = STEER_SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = STEER_CURRENT_LIMIT_ENABLE;
        motorConfig.CurrentLimits.StatorCurrentLimit = STEER_STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = STEER_CURRENT_LIMIT_ENABLE;

        // How fast can the code change torque for the motor
        motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = STEER_RAMP_RATE;

        // By Default Robot will not move
        motorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // motorConfig.Slot0.kP = 3;
        // motorConfig.Slot0.kI = 0;
        // motorConfig.Slot0.kD = 0;
        // Encoder Being Applied
        if (enableEncoder) {

            // Use the CANcoder as feedback
            motorConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
            motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        }
        // Tell it how rotor relates to module angle:
        // motorConfig.Feedback.SensorToMechanismRatio = STEER_GEAR_REDUCTION; // e.g.,
        // 12.8

        // Enable position wrapping (by default values are from 0-1)

        // MotionMagic profile config
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = STEER_CRUISE_VELOCITY;
        motorConfig.MotionMagic.MotionMagicAcceleration = STEER_ACCELERATION;

        // Apply motor config with retries (max 5 attempts)
        for (int i = 0; i < 5; i++) {
            if (motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK) {
                System.out.println("MOTOR" + motorID + "SUCCESSFULLY CONFIGURED");

                break; // Success
            }
            if (i == 4) {
                System.out.println("VERY BAD MOTOR" + motorID + "DID NOT GET CONFIGURED");
            }
        }

        // Reset motor position to 0 for consistent starting point
        motor.setPosition(0);
    }

    public void configPID(double p, double i, double d, double s) {
        Slot0Configs slot0Configs = new Slot0Configs(); // used to store and update PID values
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kS = s;

        motor.getConfigurator().apply(slot0Configs);
    }

    public SteerMotor(int motorCAN, int encoderID, CANBus canivore) {
        motorID = motorCAN;
        motor = new TalonFX(motorCAN, canivore);
        cancoder = new CANcoder(encoderID, canivore);
        configureMotor();
    }

    public void logStats() {
        Logger.recordOutput("steer/" + motorID + "/position", motor.getPosition().getValueAsDouble());
        Logger.recordOutput("steer/" + motorID + "/velocityRPM", motor.getVelocity().getValueAsDouble() * STEER_GEAR_REDUCTION * 60.0);
        Logger.recordOutput("steer/" + motorID + "/targetPosition", gurtMotorPos);
        Logger.recordOutput("steer/" + motorID + "/cancoderAbsolutePosition", cancoder.getAbsolutePosition().getValueAsDouble());
        Logger.recordOutput("steer/" + motorID + "/closedLoopReference", motor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("steer/" + motorID + "/controllerTargetRotations", controllerTargetRotations);
        Logger.recordOutput("steer/" + motorID + "/appliedVolts", motor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("steer/" + motorID + "/supplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("steer/" + motorID + "/torqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("steer/" + motorID + "/temperature", motor.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("steer/" + motorID + "/closedLoopError", motor.getClosedLoopError().getValueAsDouble());
    }

    /**
     * 
     * @param current current rotations domain: 0-1
     * @param target target rotations domain: 0-1
     * @return MotorPosition target range: -1 - 1
     */
    public double getOptimalSteerTargetPosition(double current, double target) {

        double d1 = Math.abs((target + 1) - current); // T+1
        double d2 = Math.abs((target) - current); // T
        double d3 = Math.abs((target - 1) - current);
        double motorPos = 0;
        if ((d1 <= d2) && (d1 <= d3)) {
            motorPos = target + 1;
        }
        if ((d2 <= d1) && (d2 <= d3)) {
            motorPos = target;
        }
        if ((d3 <= d1) && (d3 <= d2)) {
            motorPos = target - 1;
        }
        return motorPos;
    }

    /**
     * 
     * @param targetWheelPosition wheel position in radians, pi = 180 degrees CCW
     *        looking from the top
     */
    double controllerTargetRotations;

    public void setPosition(double targetWheelPosition) {
        gurtMotorPos = targetWheelPosition;

        targetWheelPosition = (targetWheelPosition / (2 * Math.PI)) + .5;
        controllerTargetRotations = targetWheelPosition;
        // System.out.println("moved: " + gurtMotorPos);
        gurtMotorPos = targetWheelPosition;

        targetWheelPosition = targetWheelPosition % 1;

        // radians to rotations
        // // motor.wra(motorCurrentPos);

        // targetWheelPosition = getOptimalSteerTargetPosition(motorCurrentPos,
        // targetWheelPosition);
        // targetPos = targetWheelPosition;
        posTorqueCurrentFOCRequest.withPosition(gurtMotorPos);
        motor.setControl(posTorqueCurrentFOCRequest);
    }

    /**
     * 
     * @return get position range 0-1
     */
    public double getPosition() {
        double motorCurrentPos = motor.getPosition().getValueAsDouble();
        // ensures current motor position is between 0 and 1
        return motorCurrentPos;
    }

    public double getVelocityRPM() {
        return motor.getVelocity().getValueAsDouble() * STEER_GEAR_REDUCTION * 60.0;
    }

    public void setCruiseVelocity(double velocity) {
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicCruiseVelocity = velocity;
        mmConfigs.MotionMagicAcceleration = STEER_ACCELERATION;
        motor.getConfigurator().apply(mmConfigs);
    }

    public void setCruiseVelocity(double velocity, double acceleration) {
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicCruiseVelocity = velocity;
        mmConfigs.MotionMagicAcceleration = acceleration;
        motor.getConfigurator().apply(mmConfigs);
    }
}
