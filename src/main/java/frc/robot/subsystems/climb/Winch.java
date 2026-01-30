package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;

public class Winch extends SubsystemBase {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

    private CANdi hardstopCANdi;
    private CANdiConfiguration candiConfig = new CANdiConfiguration();
    private Trigger hardstopTrigger;

    public Winch(CANBus canBusObj) {
        motor = new TalonFX(ClimbConstants.WINCH_MOTOR_CAN_ID, canBusObj);
        hardstopCANdi = new CANdi(ClimbConstants.CANDI_CAN_ID, canBusObj);
        configureCandi();
        configureMotor();

        // Reset encoder when limit switch is pressed
        hardstopTrigger = new Trigger(() -> hardstopCANdi.getS1Closed().getValue());
        hardstopTrigger.onTrue(this.runOnce(this::zeroEncoder));
    }

    private void configureMotor() {
        motorConfig.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(Amps.of(120)))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(ClimbConstants.WINCH_MOTOR_INVERTED))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(ClimbConstants.WINCH_GR))
                .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
                        .withReverseLimitEnable(true)
                        .withReverseLimitRemoteCANdiS1(hardstopCANdi))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ClimbConstants.WINCH_FORWARD_LIMIT)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ClimbConstants.WINCH_REVERSE_LIMIT));
        motor.getConfigurator().apply(motorConfig);
    }

    private void configureCandi() {
        candiConfig.withDigitalInputs(new DigitalInputsConfigs().withS1CloseState(S1CloseStateValue.CloseWhenLow));

        hardstopCANdi.getConfigurator().apply(candiConfig);
    }

    public void setMotorSpeed(double speed) {
        speed = Math.max(-1.0, Math.min(speed, 1.0));
        speed *= ClimbConstants.WINCH_MAX_SPEED;
        dutyCycleControl.withOutput(speed);
        motor.setControl(dutyCycleControl);
    }

    public double getDutyCycleSetpoint() {
        return dutyCycleControl.Output;
    }

    public void zeroEncoder() {
        motor.setPosition(0);
    }

    // hi swayam, its daniel. i'm using inline commands here because its a lot
    // easier i will move these when the code gets more complicated.
    private Command rotateWinchWithStop(double speed, BooleanSupplier stopMotor) {
        return this.startEnd(
                () -> {
                    setMotorSpeed(speed);
                }, () -> {
                    setMotorSpeed(0);
                }).until(stopMotor);
    }

    public Command pullUpClaw(BooleanSupplier stopMotor) {
        return rotateWinchWithStop(0.5, stopMotor);
    }

    public Command pullDownClaw(BooleanSupplier stopMotor) {
        return rotateWinchWithStop(-0.5, stopMotor);
    }
}