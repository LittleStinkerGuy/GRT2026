package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.LoggedTalon;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.*;

public class hood extends SubsystemBase {

    private final LoggedTalon hoodMotor;
    private final DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private PositionVoltage focThing = new PositionVoltage(0);
    private final CANcoder hoodCoder;

    private double wantedAngle = 0.1;

    private double commandedDutyCycle = 0.0;
    private static final String LOG_PREFIX = "Hood/";

    public hood(CANBus cn) {
        hoodMotor = new LoggedTalon(ShooterConstants.Hood.MOTOR_ID, cn);
        hoodCoder = new CANcoder(ShooterConstants.Hood.ENCODER_ID, cn);
        config();
    }

    public void config() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CurrentLimitsConfigs currLim = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(
                ShooterConstants.Hood.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(ShooterConstants.Hood.CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(ShooterConstants.Hood.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(ShooterConstants.Hood.CURRENT_LIMIT_ENABLE);
        cfg.withCurrentLimits(currLim);
        // cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.Hood.LOWER_ANGLE_LIMIT;
        // cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants.Hood.UPPER_ANGLE_LIMIT;
        cfg.Feedback.RotorToSensorRatio = ShooterConstants.Hood.GEAR_RATIO;

        cfg.Slot0.kP = ShooterConstants.Hood.KP;
        cfg.Slot0.kI = ShooterConstants.Hood.KI;

        CANcoderConfiguration ccfg = new CANcoderConfiguration();
        ccfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        ccfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // Use full range for absolute position

        hoodCoder.getConfigurator().apply(ccfg);

        // Use FusedCANcoder to preserve absolute position on boot (no re-zeroing)
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = ShooterConstants.Hood.ENCODER_ID;
        cfg.Feedback.SensorToMechanismRatio = 1.0; // CANcoder is 1:1 with mechanism

        hoodMotor.getConfigurator().apply(cfg);
    }

    public void setHoodAngle(double rotationAngle) {
        // if (rotationAngle >= ShooterConstants.Hood.LOWER_ANGLE_LIMIT && rotationAngle <= ShooterConstants.Hood.UPPER_ANGLE_LIMIT) {
        hoodMotor.setControl(focThing.withPosition(rotationAngle));
        System.out.println("HoodControl" + rotationAngle);
        wantedAngle = rotationAngle;
        // }
    }


    public boolean wantedAngl() {
        return Math.abs(wantedAngle - hoodMotor.getPosition().getValueAsDouble()) < ShooterConstants.Hood.ANGLE_TOLERANCE;
    }

    public void hoodSpeed(double speed) {
        commandedDutyCycle = speed;

        double pos = hoodMotor.getPosition().refresh().getValueAsDouble();
        hoodMotor.setControl(dutyCycl.withOutput(speed));

        // if (pos >= ShooterConstants.Hood.UPPER_ANGLE_LIMIT && speed < 0) {
        // hoodMotor.setControl(dutyCycl.withOutput(0));
        // } else if (pos <= ShooterConstants.Hood.LOWER_ANGLE_LIMIT && speed > 0) {
        // hoodMotor.setControl(dutyCycl.withOutput(0));
        // } else {
        // hoodMotor.setControl(dutyCycl.withOutput(speed));
        // }
    }

    public double getPos() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        hoodMotor.updateDashboard();
        sendData();
    }


    public void sendData() {
        SmartDashboard.putNumber("Shooter/Hood/HoodVelocity:", hoodMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Hood/HoodAngle:", hoodMotor.getPosition().getValueAsDouble());

        // Logger.recordOutput(LOG_PREFIX + "PositionRotations",
        // hoodMotor.getPosition().getValueAsDouble());

        // Logger.recordOutput(LOG_PREFIX + "VelocityRPS",
        // hoodMotor.getVelocity().getValueAsDouble());

        // Logger.recordOutput(LOG_PREFIX + "AppliedVolts",
        // hoodMotor.getMotorVoltage().getValueAsDouble());

        // Logger.recordOutput(LOG_PREFIX + "SupplyVoltage",
        // hoodMotor.getSupplyVoltage().getValueAsDouble());

        // Logger.recordOutput(LOG_PREFIX + "StatorCurrentAmps",
        // hoodMotor.getStatorCurrent().getValueAsDouble());

        // Logger.recordOutput(LOG_PREFIX + "SupplyCurrentAmps",
        // hoodMotor.getSupplyCurrent().getValueAsDouble());

        // Logger.recordOutput(LOG_PREFIX + "TemperatureC",
        // hoodMotor.getDeviceTemp().getValueAsDouble());

        // Logger.recordOutput(LOG_PREFIX + "CommandedDutyCycle",
        // commandedDutyCycle);

        // Logger.recordOutput(LOG_PREFIX + "Connected",
        // hoodMotor.isConnected());
    }


}
