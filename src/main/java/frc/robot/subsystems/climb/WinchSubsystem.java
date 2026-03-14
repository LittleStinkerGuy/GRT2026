package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import java.util.Optional;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMB_MECH_STATE;

public class WinchSubsystem extends SubsystemBase {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private TorqueCurrentFOC torqueCurrentControl = new TorqueCurrentFOC(0);

    private final StatusSignal<Boolean> forwardLimitSignal;
    private final StatusSignal<Boolean> reverseLimitSignal;

    private CANrange canRange;
    private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
    private Trigger homeTrigger;

    public WinchSubsystem(CANBus canBusObj) {
        motor = new TalonFX(ClimbConstants.WINCH_MOTOR_CAN_ID, canBusObj);
        canRange = new CANrange(ClimbConstants.CANRANGE_CAN_ID, canBusObj);

        configureCANrange();
        configureMotor();

        forwardLimitSignal = motor.getFault_ForwardSoftLimit();
        reverseLimitSignal = motor.getFault_ReverseSoftLimit();
        BaseStatusSignal.setUpdateFrequencyForAll(50, forwardLimitSignal, reverseLimitSignal);

        // homeTrigger = new Trigger(() -> isAtDistance(ClimbConstants.WINCH_HOME_DISTANCE));
        // homeTrigger.onTrue(this.runOnce(this::homeEncoder).ignoringDisable(true));

        // homeEncoder();
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
                .withSensorToMechanismRatio(ClimbConstants.WINCH_GR));
        // .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
        // .withForwardSoftLimitEnable(true)
        // .withForwardSoftLimitThreshold(ClimbConstants.WINCH_FORWARD_LIMIT)
        // .withReverseSoftLimitEnable(true)
        // .withReverseSoftLimitThreshold(ClimbConstants.WINCH_REVERSE_LIMIT));

        for (int i = 0; i < 5; i++) {
            if (motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK) {
                System.out.println("MOTOR " + motor.getDeviceID() + " CONFIGURED!");
                break;
            }
            if (i == 4) {
                System.out.println("VERY BAD, MOTOR " + motor.getDeviceID() + " DID NOT GET CONFIGURED");
            }
        }
    }

    private void configureCANrange() {
        for (int i = 0; i < 5; i++) {
            if (canRange.getConfigurator().apply(canRangeConfig, 0.1) == StatusCode.OK) {
                System.out.println("CANRANGE " + canRange.getDeviceID() + " CONFIGURED!");
                break;
            }
            if (i == 4) {
                System.out.println("VERY BAD, CANRANGE " + canRange.getDeviceID() + " DID NOT GET CONFIGURED");
            }
        }
    }

    private void logToDashboard() {
        SmartDashboard.putString(ClimbConstants.WINCH_TABLE + "/state", getWinchState().toString());

        SmartDashboard.putNumber(ClimbConstants.WINCH_TABLE + "/distance(mm)", getDistance().in(Millimeters));

        SmartDashboard.putNumber(ClimbConstants.WINCH_TABLE + "/dutyCycle", getDutyCycleSetpoint());
        SmartDashboard.putString(ClimbConstants.WINCH_TABLE + "/controlMode", getControlMode());

        SmartDashboard.putNumber(ClimbConstants.WINCH_TABLE + "/supplyCurrent(Amps)", getSupplyCurrent().in(Amps));
        SmartDashboard.putNumber(ClimbConstants.WINCH_TABLE + "/torqueCurrent(Amps)", getTorqueCurrent().in(Amps));

        SmartDashboard.putBoolean(ClimbConstants.WINCH_TABLE + "/forwardSoftStop", isForwardLimitActive());
        SmartDashboard.putBoolean(ClimbConstants.WINCH_TABLE + "/reverseSoftStop", isReverseLimitActive());
    }

    public void setMotorDutyCycle(double dutyCycle) {
        // Stop motor if TOF sensor reads 1cm (10mm) or less
        // if (getDistance().in(Meters) <= 0.07) {
        // dutyCycleControl.withOutput(0);
        // motor.setControl(dutyCycleControl);
        // return;
        // }

        dutyCycle = Math.max(-1.0, Math.min(dutyCycle, 1.0));
        dutyCycle *= ClimbConstants.WINCH_MAX_OUTPUT;
        dutyCycleControl.withOutput(dutyCycle);
        motor.setControl(dutyCycleControl);
    }

    public double getDutyCycleSetpoint() {
        return dutyCycleControl.Output;
    }

    public String getControlMode() {
        return motor.getControlMode().toString();
    }

    public void stop() {
        dutyCycleControl.withOutput(0);
        motor.setControl(dutyCycleControl);
    }

    public void setTorqueCurrent(double amps) {
        torqueCurrentControl.withOutput(amps);
        motor.setControl(torqueCurrentControl);
    }

    public Distance getDistance() {
        return canRange.getDistance().getValue();
    }

    public boolean isAtDistance(Distance target) {
        double errorMili = Math.abs(getDistance().in(Millimeters) - target.in(Millimeters));
        return errorMili <= ClimbConstants.WINCH_DISTANCE_TOLERANCE.in(Millimeters);
    }

    public Current getSupplyCurrent() {
        return motor.getSupplyCurrent().getValue();
    }

    public Current getTorqueCurrent() {
        return motor.getTorqueCurrent().getValue();
    }

    public void homeEncoder() {
        motor.setPosition(ClimbConstants.WINCH_HOME_POS);
    }

    public Optional<Boolean> getForwardLimit() {
        boolean forwardLimit = forwardLimitSignal.refresh().getValue();
        if (!forwardLimitSignal.hasUpdated() || forwardLimitSignal.getStatus() != StatusCode.OK) {
            return Optional.empty();
        }
        return Optional.of(forwardLimit);
    }

    public Optional<Boolean> getReverseLimit() {
        boolean reverseLimit = reverseLimitSignal.refresh().getValue();
        if (!reverseLimitSignal.hasUpdated() || reverseLimitSignal.getStatus() != StatusCode.OK) {
            return Optional.empty();
        }
        return Optional.of(reverseLimit);
    }

    public boolean isForwardLimitActive() {
        return getForwardLimit().orElse(false);
    }

    public boolean isReverseLimitActive() {
        return getReverseLimit().orElse(false);
    }

    public CLIMB_MECH_STATE getWinchState() {
        if (isAtDistance(ClimbConstants.WINCH_HOME_DISTANCE)) {
            return CLIMB_MECH_STATE.HOME;
        } else if (isAtDistance(ClimbConstants.WINCH_DEPLOYED_DISTANCE)) {
            return CLIMB_MECH_STATE.DEPLOYED;
        } else {
            return CLIMB_MECH_STATE.FLOATING;
        }
    }

    @Override
    public void periodic() {
        logToDashboard();
    }
}
