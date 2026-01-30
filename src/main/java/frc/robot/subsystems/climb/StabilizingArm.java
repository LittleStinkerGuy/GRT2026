package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class StabilizingArm extends SubsystemBase {

    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private static final double LIMIT_STATUS_FREQ_HZ = 50.0;

    private final StatusSignal<Boolean> forwardLimitSignal;
    private final StatusSignal<Boolean> reverseLimitSignal;

    public StabilizingArm(CANBus canBusObj) {
        motor = new TalonFX(ClimbConstants.ARM_MOTOR_CAN_ID, canBusObj);
        forwardLimitSignal = motor.getFault_ForwardSoftLimit();
        reverseLimitSignal = motor.getFault_ReverseSoftLimit();
        configureMotor();
        BaseStatusSignal.setUpdateFrequencyForAll(
                LIMIT_STATUS_FREQ_HZ, forwardLimitSignal, reverseLimitSignal);
        motor.optimizeBusUtilization(0, 1.0);

        setEncoder(Rotations.of(0));
    }

    private void configureMotor() {
        motorConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(Amps.of(120)))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(ClimbConstants.ARM_MOTOR_INVERTED))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(ClimbConstants.ARM_GR))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ClimbConstants.ARM_FORWARD_LIMIT)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ClimbConstants.ARM_REVERSE_LIMIT));
        motor.getConfigurator().apply(motorConfig);
    }

    public void setMotorSpeed(double speed) {
        speed = Math.max(-1.0, Math.min(speed, 1.0));
        speed *= ClimbConstants.ARM_MAX_SPEED;
        dutyCycleControl.withOutput(speed);
        motor.setControl(dutyCycleControl);
    }

    public double getDutyCycleSetpoint() {
        return dutyCycleControl.Output;
    }

    public void zeroEncoder() {
        motor.setPosition(0);
    }

    public void setEncoder(Angle pos) {
        motor.setPosition(pos);
    }

    public boolean getForwardLimit() {
        if (!forwardLimitSignal.refresh().getValue()) {
            return false;
        }
        return forwardLimitSignal.getValue();
    }

    public boolean getReverseLimit() {
        if (!reverseLimitSignal.refresh().getValue()) {
            return false;
        }
        return reverseLimitSignal.getValue();
    }

    // hi swayam, its daniel. i'm using inline commands here because its a lot
    // easier i will move these when the code gets more complicated.
    private Command moveArmWithStop(double speed, BooleanSupplier stopMotor) {
        return this.startEnd(
                () -> {
                    setMotorSpeed(speed);
                }, () -> {
                    setMotorSpeed(0);
                }).until(stopMotor);
    }

    public Command deployArm(BooleanSupplier stopMotor) {
        return moveArmWithStop(1, stopMotor);
    }

    public Command retractArm(BooleanSupplier stopMotor) {
        return moveArmWithStop(-1, stopMotor);
    }
}
