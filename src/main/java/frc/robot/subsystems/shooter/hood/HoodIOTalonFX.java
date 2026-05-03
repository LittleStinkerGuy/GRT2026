package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {
    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(ShooterConstants.Hood.kP)
        .withKI(ShooterConstants.Hood.kI)
        .withKD(ShooterConstants.Hood.kD)
        .withKS(ShooterConstants.Hood.kS);

    protected final TalonFX motor;
    protected final CANcoder cancoder;

    private final Slot0Configs pidConfig;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);
    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);

    private final Alert failedToSetMotorSignalFrequencyAlert = new Alert("Hood", "Failed to set motor status signal frequency!", AlertType.kError);
    private final Alert failedToConfigureMotorAlert = new Alert("Hood", "Failed to configure hood motor!", AlertType.kError);

    private final Alert cancoderConfigRefreshAlert = new Alert("Pivot", "Failed to refresh cancoder config", AlertType.kError);
    private final Alert cancoderConfigAlert = new Alert("Hood", "Failed to configure cancoder", AlertType.kError);
    private final Alert failedToSetCancoderSignalFrequencyAlert = new Alert("Hood", "Failed to set cancoder status signal frequency!", AlertType.kError);

    private final Alert didNotOptimizeMotorCANAlert = new Alert("Hood", "Didn't optimize motor CAN", AlertType.kWarning);
    private final Alert didNotOptimizeCancoderCANAlert = new Alert("Hood", "Didn't optimize cancoder CAN", AlertType.kWarning);
    private final Alert pidNotSetAlert = new Alert("Hood", "Motor PID was not saved", AlertType.kWarning);

    private final List<BaseStatusSignal> motorSignals;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<AngularAcceleration> accel;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temp;
    private final StatusSignal<Boolean> tempFault;
    private final StatusSignal<ControlModeValue> controlMode;
    private final StatusSignal<Double> appliedDutyCycle;
    private final StatusSignal<Double> closedLoopReference;
    private final StatusSignal<Double> closedLoopOutput;

    private final List<BaseStatusSignal> cancoderSignals;
    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<MagnetHealthValue> magnetHealth;

    public HoodIOTalonFX(LoggedCanivore canivore) {
        motor = new TalonFX(ShooterConstants.Hood.MOTOR_ID, canivore);
        cancoder = new CANcoder(ShooterConstants.Hood.ENCODER_ID, canivore);

        // Cancoder Configuration
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        tryUntilOk(5, () -> cancoder.getConfigurator().refresh(cancoderConfig), cancoderConfigRefreshAlert);

        double midRotation = (ShooterConstants.Hood.UPPER_ANGLE_LIMIT.in(Rotations)
            + ShooterConstants.Hood.LOWER_ANGLE_LIMIT.in(Rotations)) / 2.0;
        double discontinuityRotation = ((midRotation + 0.5) % 1.0 + 1.0) % 1.0;
        Angle discontinuityPoint = Rotations.of(discontinuityRotation);
        cancoderConfig.withMagnetSensor(cancoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(discontinuityPoint));
        tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig), cancoderConfigAlert);

        absolutePosition = cancoder.getAbsolutePosition(false);
        magnetHealth = cancoder.getMagnetHealth(false);
        cancoderSignals = List.of(absolutePosition, magnetHealth);

        tryUntilOk(5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(120.0, cancoderSignals),
            failedToSetCancoderSignalFrequencyAlert);
        tryUntilOk(5, () -> cancoder.optimizeBusUtilization(0, 1.0), didNotOptimizeCancoderCANAlert);
        PhoenixUtil.registerSignals(canivore.getCanType(), cancoderSignals);

        // Motor Configuration
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

        pidConfig = new Slot0Configs()
            .withKP(ShooterConstants.Hood.kP)
            .withKI(ShooterConstants.Hood.kI)
            .withKD(ShooterConstants.Hood.kD)
            .withKS(ShooterConstants.Hood.kS);
        config.withSlot0(pidConfig);

        config.withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ShooterConstants.Hood.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(ShooterConstants.Hood.CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(ShooterConstants.Hood.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(ShooterConstants.Hood.CURRENT_LIMIT_ENABLE));

        config.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ShooterConstants.Hood.UPPER_ANGLE_LIMIT)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ShooterConstants.Hood.LOWER_ANGLE_LIMIT));

        // Preserve sign-flip from the original implementation: encoder counts go
        // opposite the desired mechanism convention, so invert via the
        // SensorToMechanismRatio while sourcing position from the remote CANcoder.
        config.withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(cancoder.getDeviceID())
            .withRotorToSensorRatio(-1.0 * ShooterConstants.Hood.GEAR_RATIO)
            .withSensorToMechanismRatio(-1.0));

        tryUntilOk(5, () -> motor.getConfigurator().apply(config), failedToConfigureMotorAlert);

        position = motor.getPosition(false);
        velocity = motor.getVelocity(false);
        accel = motor.getAcceleration(false);
        appliedVoltage = motor.getMotorVoltage(false);
        supplyCurrent = motor.getSupplyCurrent(false);
        statorCurrent = motor.getStatorCurrent(false);
        torqueCurrent = motor.getTorqueCurrent(false);
        temp = motor.getDeviceTemp(false);
        tempFault = motor.getFault_DeviceTemp(false);
        controlMode = motor.getControlMode(false);
        appliedDutyCycle = motor.getDutyCycle(false);
        closedLoopReference = motor.getClosedLoopReference(false);
        closedLoopOutput = motor.getClosedLoopOutput(false);
        motorSignals = List.of(
            position,
            velocity,
            accel,
            appliedVoltage,
            supplyCurrent,
            statorCurrent,
            torqueCurrent,
            temp,
            tempFault,
            controlMode,
            appliedDutyCycle,
            closedLoopReference,
            closedLoopOutput);

        tryUntilOk(5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(120.0, motorSignals),
            failedToSetMotorSignalFrequencyAlert);
        tryUntilOk(5, () -> motor.optimizeBusUtilization(0, 1.0), didNotOptimizeMotorCANAlert);
        PhoenixUtil.registerSignals(canivore.getCanType(), motorSignals);
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.acceleration = accel.getValue();
        inputs.appliedVoltage = appliedVoltage.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.temp = temp.getValue();
        inputs.tempFault = tempFault.getValue();
        inputs.motorConnected = BaseStatusSignal.isAllGood(motorSignals);

        inputs.encoderAbsolutePosition = absolutePosition.getValue();
        inputs.encoderHealth = PhoenixUtil.toEncoderHealth(magnetHealth.getValue());
        inputs.encoderConnected = BaseStatusSignal.isAllGood(cancoderSignals);

        inputs.controlMode = PhoenixUtil.toMotorControlMode(controlMode.getValue());
        inputs.appliedDutyCycle = appliedDutyCycle.getValue();
        inputs.closedLoopSetpoint = closedLoopReference.getValue();
        inputs.closedLoopOutput = closedLoopOutput.getValue();
    }

    @Override
    public void updatePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        pidConfig.withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV).withKA(kA);
        tryUntilOk(5, () -> motor.getConfigurator().apply(pidConfig), pidNotSetAlert);
    }

    @Override
    public void setDutyCycleOut(double dutyCycle) {
        motor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setVoltageOut(Voltage voltsOut) {
        motor.setControl(voltageControl.withOutput(voltsOut));
    }

    @Override
    public void setPositionOut(Angle desiredAngle) {
        motor.setControl(positionControl.withPosition(desiredAngle));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
