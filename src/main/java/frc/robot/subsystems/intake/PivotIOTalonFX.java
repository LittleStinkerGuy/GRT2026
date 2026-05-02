package frc.robot.subsystems.intake;

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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;
import frc.robot.util.PhoenixUtil;

public class PivotIOTalonFX implements PivotIO {
    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(IntakeConstants.PIVOT_P)
        .withKI(IntakeConstants.PIVOT_I)
        .withKD(IntakeConstants.PIVOT_D)
        .withKS(IntakeConstants.PIVOT_S)
        .withKV(IntakeConstants.PIVOT_V)
        .withKA(IntakeConstants.PIVOT_A);

    protected final TalonFX motor;
    private final Slot0Configs pidConfig;

    protected final CANcoder cancoder;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);
    private final PositionVoltage positionControl = new PositionVoltage(0).withEnableFOC(true);

    private final Alert failedToSetMotorSignalFrequencyAlert = new Alert("Pivot", "Failed to set motor status signal frequency!", AlertType.kError);
    private final Alert failedToConfigureMotorAlert = new Alert("Pivot", "Failed to configure pivot motor!", AlertType.kError);

    private final Alert cancoderConfigRefreshAlert = new Alert("Pivot", "Failed to refresh cancoder config", AlertType.kError);
    private final Alert cancoderConfigAlert = new Alert("Pivot", "Failed to configure cancoder", AlertType.kError);
    private final Alert failedToSetCancoderSignalFrequencyAlert = new Alert("Pivot", "Failed to set cancoder status signal frequency!", AlertType.kError);

    private final Alert didNotOptimizeMotorCANAlert = new Alert("Pivot", "Didn't optimize motor CAN", AlertType.kWarning);
    private final Alert didNotOptimizeCancoderCANAlert = new Alert("Pivot", "Didn't optimize cancoder CAN", AlertType.kWarning);
    private final Alert pidNotSetAlert = new Alert("Pivot", "Motor PID was not saved", AlertType.kWarning);

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

    public PivotIOTalonFX(LoggedCanivore canivore) {
        motor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, canivore);
        cancoder = new CANcoder(IntakeConstants.PIVOT_CANCODER_ID, canivore);

        // Cancoder Configuration
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        tryUntilOk(5, () -> cancoder.getConfigurator().refresh(cancoderConfig), cancoderConfigRefreshAlert);

        double midRotation = (IntakeConstants.PIVOT_FORWARD_LIMIT.in(Rotations)
            + IntakeConstants.PIVOT_REVERSE_LIMIT.in(Rotations)) / 2.0;
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

        // PID Config
        pidConfig = new Slot0Configs()
            .withKP(IntakeConstants.PIVOT_P)
            .withKI(IntakeConstants.PIVOT_I)
            .withKD(IntakeConstants.PIVOT_D)
            .withKS(IntakeConstants.PIVOT_S)
            .withKV(IntakeConstants.PIVOT_V)
            .withKA(IntakeConstants.PIVOT_A);
        config.withSlot0(pidConfig);

        // Motor Output Config
        config.withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        // Using internal encoder only (FusedCANcoder disabled for sysID)
        config.withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(cancoder.getDeviceID())
            .withRotorToSensorRatio(IntakeConstants.GEAR_RATIO)
            .withSensorToMechanismRatio(1.0));

        // StatorCurrent Limits
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT_ENABLE));

        // Software Limits
        config.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(IntakeConstants.PIVOT_FORWARD_LIMIT)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(IntakeConstants.PIVOT_REVERSE_LIMIT));

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
    public void updateInputs(PivotIOInputs inputs) {
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
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
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
