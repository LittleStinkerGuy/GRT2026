package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMB_MECH_STATE;

public class WinchSubsystem extends SubsystemBase {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private PositionTorqueCurrentFOC posControl = new PositionTorqueCurrentFOC(0).withSlot(0);

    private final StatusSignal<Boolean> forwardLimitSignal;
    private final StatusSignal<Boolean> reverseLimitSignal;

    private CANdi hardstopCANdi;
    private CANdiConfiguration candiConfig = new CANdiConfiguration();
    private Trigger hardstopTrigger;

    public WinchSubsystem(CANBus canBusObj) {
        motor = new TalonFX(ClimbConstants.WINCH_MOTOR_CAN_ID, canBusObj);

        forwardLimitSignal = motor.getFault_ForwardSoftLimit();
        reverseLimitSignal = motor.getFault_ReverseSoftLimit();

        hardstopCANdi = new CANdi(ClimbConstants.CANDI_CAN_ID, canBusObj);
        configureCandi();
        configureMotor();

        // Reset encoder when limit switch is pressed
        hardstopTrigger = new Trigger(() -> hardstopCANdi.getS1Closed().getValue());
        hardstopTrigger.onTrue(this.runOnce(this::homeEncoder).ignoringDisable(true));

        // Change soft limit signal update frequency
        // idk why this is necessary but it makes code work
        BaseStatusSignal.setUpdateFrequencyForAll(50, forwardLimitSignal, reverseLimitSignal);

        homeEncoder();
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
                        .withReverseSoftLimitThreshold(ClimbConstants.WINCH_REVERSE_LIMIT))
                .withSlot0(new Slot0Configs()
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(ClimbConstants.WINCH_kP)
                        .withKI(ClimbConstants.WINCH_kI)
                        .withKD(ClimbConstants.WINCH_kD)
                        .withKG(ClimbConstants.WINCH_kG)
                        .withKS(ClimbConstants.WINCH_kS));

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

    private void configureCandi() {
        candiConfig.withDigitalInputs(new DigitalInputsConfigs().withS1CloseState(S1CloseStateValue.CloseWhenLow));

        for (int i = 0; i < 5; i++) {
            if (hardstopCANdi.getConfigurator().apply(candiConfig, 0.1) == StatusCode.OK) {
                System.out.println("CANDI " + hardstopCANdi.getDeviceID() + " CONFIGURED!");
                break; // Success
            }
            if (i == 4) {
                System.out.println("VERY BAD, CANDI " + hardstopCANdi.getDeviceID() + " DID NOT GET CONFIGURED");
            }
        }
    }

    private void logToDashboard() {
        SmartDashboard.putString(ClimbConstants.WINCH_TABLE + "/state", getWinchState().toString());

        SmartDashboard.putNumber(ClimbConstants.WINCH_TABLE + "/rotations", getMotorPosition().in(Rotations));
        SmartDashboard.putNumber(ClimbConstants.WINCH_TABLE + "/setRotations", getPositionSetpoint().in(Rotations));
        SmartDashboard.putBoolean(ClimbConstants.WINCH_TABLE + "/atSetpoint", atSetPosition());

        SmartDashboard.putNumber(ClimbConstants.WINCH_TABLE + "/dutyCycle", getDutyCycleSetpoint());

        SmartDashboard.putBoolean(ClimbConstants.WINCH_TABLE + "/reverseHardStop", hardstopTrigger.getAsBoolean());
        SmartDashboard.putBoolean(ClimbConstants.WINCH_TABLE + "/forwardSoftStop",
                getForwardLimit().orElse(false));
        SmartDashboard.putBoolean(ClimbConstants.WINCH_TABLE + "/reverseSoftStop",
                getReverseLimit().orElse(false));
    }

    // take an input value and clamp it to the max value then run motor at that duty
    // cycle
    public void setMotorDutyCycle(double dutyCycle) {
        dutyCycle = Math.max(-1.0, Math.min(dutyCycle, 1.0));
        dutyCycle *= ClimbConstants.WINCH_MAX_OUTPUT;
        dutyCycleControl.withOutput(dutyCycle);
        motor.setControl(dutyCycleControl);
    }

    public double getDutyCycleSetpoint() {
        return dutyCycleControl.Output;
    }

    public void setPositionSetpoint(Angle setpoint) {
        if (setpoint.gt(ClimbConstants.WINCH_FORWARD_LIMIT)) {
            setpoint = ClimbConstants.WINCH_FORWARD_LIMIT;
        } else if (setpoint.lt(ClimbConstants.WINCH_REVERSE_LIMIT)) {
            setpoint = ClimbConstants.WINCH_REVERSE_LIMIT;
        }

        posControl.withPosition(setpoint);
        motor.setControl(posControl);
    }

    public Angle getPositionSetpoint() {
        if (motor.getControlMode().getValue() != ControlModeValue.PositionTorqueCurrentFOC) {
            return null;
        }
        return Rotations.of(motor.getClosedLoopReference().getValue());
    }

    // Checks if arm is at position set by PID control with a tolerance
    public boolean atSetPosition() {
        // if not in position control return false
        if (motor.getControlMode().getValue() != ControlModeValue.PositionTorqueCurrentFOC) {
            return false;
        }

        // checks if closed loop error is within tolerance
        return (Rotations.of(Math.abs(motor.getClosedLoopError().getValue())))
                .lte(ClimbConstants.WINCH_ACCEPTABLE_POSITION_ERROR);
    }

    // Checks if arm is at given position
    public boolean atPosition(Angle target) {
        // finds error between position and target and its absolute value
        Angle error = target.minus(getMotorPosition());
        Angle absError = Radians.of(Math.abs(error.in(Radians)));

        // checks if difference is within tolerance
        return absError.lte(ClimbConstants.WINCH_ACCEPTABLE_POSITION_ERROR);
    }

    public void homeEncoder() {
        setEncoder(ClimbConstants.WINCH_HOME_POS);
    }

    public void zeroEncoder() {
        setEncoder(Rotations.of(0));
    }

    public void setEncoder(Angle pos) {
        motor.setPosition(pos);
    }

    public Angle getMotorPosition() {
        return motor.getPosition().getValue();
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

    public CLIMB_MECH_STATE getWinchState() {
        if (atPosition(ClimbConstants.WINCH_HOME_POS)) {
            return CLIMB_MECH_STATE.HOME;
        } else if (atPosition(ClimbConstants.WINCH_DEPLOYED_POS)) {
            return CLIMB_MECH_STATE.DEPLOYED;
        } else {
            return CLIMB_MECH_STATE.FLOATING;
        }
    }

    public boolean isForwardLimitActive() {
        return getForwardLimit().orElse(false);
    }

    public boolean isReverseLimitActive() {
        return getReverseLimit().orElse(false);
    }

    public boolean isHardstopPressed() {
        return hardstopTrigger.getAsBoolean();
    }
}
