package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
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

    private final StatusSignal<Boolean> forwardLimitSignal;
    private final StatusSignal<Boolean> reverseLimitSignal;

    public StabilizingArm(CANBus canBusObj) {
        motor = new TalonFX(ClimbConstants.ARM_MOTOR_CAN_ID, canBusObj);
        forwardLimitSignal = motor.getFault_ForwardSoftLimit();
        reverseLimitSignal = motor.getFault_ReverseSoftLimit();
        configureMotor();

        zeroEncoder();

        // Change soft limit signal update frequency
        // idk why this is necessary but it makes code work
        BaseStatusSignal.setUpdateFrequencyForAll(50, forwardLimitSignal, reverseLimitSignal);
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

        for (int i = 0; i < 5; i++) {
            if (motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK) {
                System.out.println("MOTOR " + motor.getDeviceID() + " CONFIGURED!");
                break; // Success
            }
            if (i == 4) {
                System.out.println("VERY BAD, MOTOR " + motor.getDeviceID() + " DID NOT GET CONFIGURED");
            }
        }
    }

    // take an input value and clamp it to the max value then run motor at that duty
    // cycle
    public void setMotorDutyCycle(double targetOutput) {
        targetOutput = Math.max(-1.0, Math.min(targetOutput, 1.0));
        var dutyCycle = targetOutput * ClimbConstants.ARM_MAX_OUTPUT;

        dutyCycleControl.withOutput(dutyCycle);
        motor.setControl(dutyCycleControl);
    }

    public double getDutyCycleSetpoint() {
        return dutyCycleControl.Output;
    }

    public void setEncoder(Angle pos) {
        motor.setPosition(pos);
    }

    public void zeroEncoder() {
        setEncoder(Rotations.of(0));
    }

    // returns false if can't refresh
    public Optional<Boolean> getForwardLimit() {
        if (!forwardLimitSignal.refresh().getValue()) {
            return Optional.empty();
        }
        return Optional.of(forwardLimitSignal.getValue());
    }

    // returns false if can't refresh
    public Optional<Boolean> getReverseLimit() {
        if (!reverseLimitSignal.refresh().getValue()) {
            return Optional.empty();
        }
        return Optional.of(reverseLimitSignal.getValue());
    }

    // hi swayam, its daniel. i'm using inline commands here because its a lot
    // easier i will move these when the code gets more complicated.

    // rotate motor and stop it when boolean is true
    private Command moveArmWithStop(double dutyCycle, BooleanSupplier stopMotor) {
        return this.startEnd(
                () -> {
                    setMotorDutyCycle(dutyCycle);
                }, () -> {
                    setMotorDutyCycle(0);
                }).until(stopMotor);
    }

    // make arm go down and stop with boolean supplier
    public Command deployArm(BooleanSupplier stopMotor) {
        return moveArmWithStop(1, stopMotor);
    }

    // make arm go up and stop with boolean supplier
    public Command retractArm(BooleanSupplier stopMotor) {
        return moveArmWithStop(-1, stopMotor);
    }
}
