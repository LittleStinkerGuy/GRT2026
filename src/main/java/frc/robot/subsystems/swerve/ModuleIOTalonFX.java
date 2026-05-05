package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import java.util.List;
import java.util.Queue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveSteerConstants;
import frc.robot.subsystems.swerve.DriveSubsystem.SwerveModule;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;
import frc.robot.util.PhoenixUtil;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final Slot0Configs drivePIDConfig;

    private final TalonFX steerMotor;
    private final Slot0Configs steerPIDConfig;

    private final CANcoder steerEncoder;

    private final PositionVoltage positionControl = new PositionVoltage(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0.0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);

    private final List<BaseStatusSignal> driveSignals;
    private StatusSignal<Angle> drivePosition;
    private StatusSignal<AngularVelocity> driveVelocity;
    private StatusSignal<AngularAcceleration> driveAcceleration;
    private StatusSignal<Voltage> driveAppliedVoltage;
    private StatusSignal<Current> driveSupplyCurrent;
    private StatusSignal<Current> driveTorqueCurrent;
    private StatusSignal<Current> driveStatorCurrent;
    private StatusSignal<Temperature> driveTemp;
    private StatusSignal<Boolean> driveTempFault;
    private StatusSignal<ControlModeValue> driveControlMode;
    private StatusSignal<Double> driveAppliedDutyCycle;
    private StatusSignal<Double> driveClosedLoopSetpoint;
    private StatusSignal<Double> driveClosedLoopOutput;

    private final List<BaseStatusSignal> steerSignals;
    private StatusSignal<Angle> steerPosition;
    private StatusSignal<AngularVelocity> steerVelocity;
    private StatusSignal<AngularAcceleration> steerAcceleration;
    private StatusSignal<Voltage> steerAppliedVoltage;
    private StatusSignal<Current> steerSupplyCurrent;
    private StatusSignal<Current> steerTorqueCurrent;
    private StatusSignal<Current> steerStatorCurrent;
    private StatusSignal<Temperature> steerTemp;
    private StatusSignal<Boolean> steerTempFault;
    private StatusSignal<ControlModeValue> steerControlMode;
    private StatusSignal<Double> steerAppliedDutyCycle;
    private StatusSignal<Double> steerClosedLoopSetpoint;
    private StatusSignal<Double> steerClosedLoopOutput;

    private final List<BaseStatusSignal> cancoderSignals;
    private StatusSignal<Angle> cancoderAbsolutePosition;
    private StatusSignal<MagnetHealthValue> cancoderHealth;

    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> steerPositionQueue;

    private final List<BaseStatusSignal> odometrySignals;

    private final Alert failedToConfigureDrive;
    private final Alert failedToSetDriveFrequencyAlert;
    private final Alert drivePIDNotSetAlert;

    private final Alert failedToConfigureSteer;
    private final Alert failedToSetSteerFrequencyAlert;
    private final Alert steerPIDNotSetAlert;

    private final Alert cancoderConfigRefreshAlert;
    private final Alert cancoderConfigAlert;
    private final Alert failedToSetCancoderSignalFrequencyAlert;

    private final Alert failedToSetOdometrySignalFrequencyAlert;
    private final Alert didNotOptimizeCanBusesAlert;

    public ModuleIOTalonFX(SwerveModule module, int driveMotorID, int steerMotorID, int cancoderID, LoggedCanivore canivore, PIDConstants drivePID, PIDConstants steerPID) {
        String driveDescription = "Drive Motor " + driveMotorID + " in " + module.toString();
        String steerDescription = "Steer Motor " + steerMotorID + " in " + module.toString();
        String cancoderDescription = "Cancoder " + cancoderID + " in " + module.toString();

        failedToConfigureDrive = new Alert(
            "Swerve",
            "Failed to configure " + driveDescription,
            AlertType.kError);
        failedToSetDriveFrequencyAlert = new Alert(
            "Swerve",
            "Failed to set status signal frequency for " + driveDescription,
            AlertType.kError);
        drivePIDNotSetAlert = new Alert(
            "Swerve",
            "PID not set for " + driveDescription,
            AlertType.kWarning);

        failedToConfigureSteer = new Alert(
            "Swerve",
            "Failed to configure " + steerDescription,
            AlertType.kError);
        failedToSetSteerFrequencyAlert = new Alert(
            "Swerve",
            "Failed to set status signal frequency for " + steerDescription,
            AlertType.kError);
        steerPIDNotSetAlert = new Alert(
            "Swerve",
            "PID not set for " + steerDescription,
            AlertType.kWarning);

        cancoderConfigRefreshAlert = new Alert(
            "Swerve",
            "Failed to refresh config for " + cancoderDescription,
            AlertType.kError);
        cancoderConfigAlert = new Alert(
            "Swerve",
            "Failed to configure " + cancoderDescription,
            AlertType.kError);
        failedToSetCancoderSignalFrequencyAlert = new Alert(
            "Swerve",
            "Failed to set status signal frequency for " + cancoderDescription,
            AlertType.kError);

        failedToSetOdometrySignalFrequencyAlert = new Alert(
            "Swerve",
            "Failed to set odometry signal frequency for " + module.toString(),
            AlertType.kError);
        didNotOptimizeCanBusesAlert = new Alert(
            "Swerve",
            "Failed to optimize CAN for " + module.toString(),
            AlertType.kWarning);

        driveMotor = new TalonFX(driveMotorID, canivore);
        steerMotor = new TalonFX(steerMotorID, canivore);
        steerEncoder = new CANcoder(cancoderID, canivore);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.withTorqueCurrent(new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(SwerveDriveConstants.DRIVE_PEAK_STATOR_CURRENT)
            .withPeakReverseTorqueCurrent(-SwerveDriveConstants.DRIVE_PEAK_STATOR_CURRENT));
        driveConfig.withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(SwerveDriveConstants.DRIVE_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(SwerveDriveConstants.DRIVE_CURRENT_LIMIT_ENABLE)
            .withStatorCurrentLimit(SwerveDriveConstants.DRIVE_STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(SwerveDriveConstants.DRIVE_CURRENT_LIMIT_ENABLE));

        driveConfig.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        driveConfig.withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(SwerveDriveConstants.DRIVE_GEAR_REDUCTION));
        drivePIDConfig = new Slot0Configs()
            .withKP(drivePID.kP())
            .withKI(drivePID.kI())
            .withKD(drivePID.kD());
        driveConfig.withSlot0(drivePIDConfig);
        tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig), failedToConfigureDrive);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.withTorqueCurrent(new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(SwerveSteerConstants.STEER_PEAK_STATOR_CURRENT)
            .withPeakReverseTorqueCurrent(-SwerveSteerConstants.STEER_PEAK_STATOR_CURRENT));
        steerConfig.withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(SwerveSteerConstants.STEER_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(SwerveSteerConstants.STEER_CURRENT_LIMIT_ENABLE)
            .withStatorCurrentLimit(SwerveSteerConstants.STEER_STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(SwerveSteerConstants.STEER_CURRENT_LIMIT_ENABLE));
        steerConfig.withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
        steerConfig.withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(cancoderID)
            .withRotorToSensorRatio(SwerveSteerConstants.STEER_GEAR_REDUCTION));
        steerConfig.withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
            .withContinuousWrap(true));
        steerPIDConfig = new Slot0Configs()
            .withKP(steerPID.kP())
            .withKI(steerPID.kI())
            .withKD(steerPID.kD())
            .withKS(steerPID.kS())
            .withKV(steerPID.kV());
        steerConfig.withSlot0(steerPIDConfig);
        tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerConfig), failedToConfigureSteer);

        // Cancoder Configuration
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        tryUntilOk(5, () -> steerEncoder.getConfigurator().refresh(cancoderConfig), cancoderConfigRefreshAlert);

        cancoderConfig.withMagnetSensor(cancoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        tryUntilOk(5, () -> steerEncoder.getConfigurator().apply(cancoderConfig), cancoderConfigAlert);

        drivePosition = driveMotor.getPosition(false);
        driveVelocity = driveMotor.getVelocity(false);
        driveAcceleration = driveMotor.getAcceleration(false);
        driveAppliedVoltage = driveMotor.getMotorVoltage(false);
        driveSupplyCurrent = driveMotor.getSupplyCurrent(false);
        driveTorqueCurrent = driveMotor.getTorqueCurrent(false);
        driveStatorCurrent = driveMotor.getStatorCurrent(false);
        driveTemp = driveMotor.getDeviceTemp(false);
        driveTempFault = driveMotor.getFault_DeviceTemp(false);
        driveControlMode = driveMotor.getControlMode(false);
        driveAppliedDutyCycle = driveMotor.getDutyCycle(false);
        driveClosedLoopSetpoint = driveMotor.getClosedLoopReference(false);
        driveClosedLoopOutput = driveMotor.getClosedLoopOutput(false);
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition(false).clone());

        driveSignals = List.of(
            drivePosition,
            driveVelocity,
            driveAcceleration,
            driveAppliedVoltage,
            driveSupplyCurrent,
            driveTorqueCurrent,
            driveStatorCurrent,
            driveTemp,
            driveTempFault,
            driveControlMode,
            driveAppliedDutyCycle,
            driveClosedLoopSetpoint,
            driveClosedLoopOutput);

        steerPosition = steerMotor.getPosition(false);
        steerVelocity = steerMotor.getVelocity(false);
        steerAcceleration = steerMotor.getAcceleration(false);
        steerAppliedVoltage = steerMotor.getMotorVoltage(false);
        steerSupplyCurrent = steerMotor.getSupplyCurrent(false);
        steerTorqueCurrent = steerMotor.getTorqueCurrent(false);
        steerStatorCurrent = steerMotor.getStatorCurrent(false);
        steerTemp = steerMotor.getDeviceTemp(false);
        steerTempFault = steerMotor.getFault_DeviceTemp(false);
        steerControlMode = steerMotor.getControlMode(false);
        steerAppliedDutyCycle = steerMotor.getDutyCycle(false);
        steerClosedLoopSetpoint = steerMotor.getClosedLoopReference(false);
        steerClosedLoopOutput = steerMotor.getClosedLoopOutput(false);
        steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerMotor.getPosition(false).clone());

        steerSignals = List.of(
            steerPosition,
            steerVelocity,
            steerAcceleration,
            steerAppliedVoltage,
            steerSupplyCurrent,
            steerTorqueCurrent,
            steerStatorCurrent,
            steerTemp,
            steerTempFault,
            steerControlMode,
            steerAppliedDutyCycle,
            steerClosedLoopSetpoint,
            steerClosedLoopOutput);

        cancoderAbsolutePosition = steerEncoder.getAbsolutePosition(false);
        cancoderHealth = steerEncoder.getMagnetHealth(false);
        cancoderSignals = List.of(cancoderAbsolutePosition, cancoderHealth);

        odometrySignals = List.of(drivePosition, steerPosition);

        tryUntilOk(
            5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(120.0, driveSignals),
            failedToSetDriveFrequencyAlert);
        tryUntilOk(
            5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(120.0, steerSignals),
            failedToSetSteerFrequencyAlert);
        tryUntilOk(
            5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(120.0, cancoderSignals),
            failedToSetCancoderSignalFrequencyAlert);
        tryUntilOk(
            5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(250, odometrySignals),
            failedToSetOdometrySignalFrequencyAlert);
        tryUntilOk(
            5,
            () -> ParentDevice.optimizeBusUtilizationForAll(0.0, driveMotor, steerMotor, steerEncoder), didNotOptimizeCanBusesAlert);

        PhoenixUtil.registerSignals(canivore.getCanType(), driveSignals);
        PhoenixUtil.registerSignals(canivore.getCanType(), steerSignals);
        PhoenixUtil.registerSignals(canivore.getCanType(), cancoderSignals);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePosition = drivePosition.getValue();
        inputs.driveVelocity = driveVelocity.getValue();
        inputs.driveAcceleration = driveAcceleration.getValue();
        inputs.driveAppliedVoltage = driveAppliedVoltage.getValue();
        inputs.driveSupplyCurrent = driveSupplyCurrent.getValue();
        inputs.driveTorqueCurrent = driveTorqueCurrent.getValue();
        inputs.driveStatorCurrent = driveStatorCurrent.getValue();
        inputs.driveTemp = driveTemp.getValue();
        inputs.driveTempFault = driveTempFault.getValue();
        inputs.driveMotorConnected = BaseStatusSignal.isAllGood(driveSignals);

        inputs.driveControlMode = PhoenixUtil.toMotorControlMode(driveControlMode.getValue());
        inputs.driveAppliedDutyCycle = driveAppliedDutyCycle.getValue();
        inputs.driveClosedLoopSetpoint = driveClosedLoopSetpoint.getValue();
        inputs.driveClosedLoopOutput = driveClosedLoopOutput.getValue();

        inputs.steerPosition = steerPosition.getValue();
        inputs.steerVelocity = steerVelocity.getValue();
        inputs.steerAcceleration = steerAcceleration.getValue();
        inputs.steerAppliedVoltage = steerAppliedVoltage.getValue();
        inputs.steerSupplyCurrent = steerSupplyCurrent.getValue();
        inputs.steerTorqueCurrent = steerTorqueCurrent.getValue();
        inputs.steerStatorCurrent = steerStatorCurrent.getValue();
        inputs.steerTemp = steerTemp.getValue();
        inputs.steerTempFault = steerTempFault.getValue();
        inputs.steerMotorConnected = BaseStatusSignal.isAllGood(steerSignals);

        inputs.steerControlMode = PhoenixUtil.toMotorControlMode(steerControlMode.getValue());
        inputs.steerAppliedDutyCycle = steerAppliedDutyCycle.getValue();
        inputs.steerClosedLoopSetpoint = steerClosedLoopSetpoint.getValue();
        inputs.steerClosedLoopOutput = steerClosedLoopOutput.getValue();

        inputs.encoderAbsolutePosition = cancoderAbsolutePosition.getValue();
        inputs.encoderHealth = PhoenixUtil.toEncoderHealth(cancoderHealth.getValue());
        inputs.encoderConnected = BaseStatusSignal.isAllGood(cancoderSignals);

        inputs.odometryDrivePositionsRads = drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
        inputs.odometrySteerPositions = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity, Voltage feedforward) {
        driveMotor.setControl(velocityControl.withVelocity(velocity).withFeedForward(feedforward));
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        setDriveVelocity(velocity, Volts.of(0.0));
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        driveMotor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void setSteerPosition(Angle rotation) {
        steerMotor.setControl(positionControl.withPosition(rotation));
    }

    @Override
    public void setSteerVoltage(Voltage voltage) {
        steerMotor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void stopSteer() {
        steerMotor.stopMotor();
    }

    @Override
    public void stopDrive() {
        driveMotor.stopMotor();
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        drivePIDConfig.withKP(kP).withKI(kI).withKD(kD);
        tryUntilOk(5, () -> driveMotor.getConfigurator().apply(drivePIDConfig), drivePIDNotSetAlert);
    }

    @Override
    public void setSteerPID(double kP, double kI, double kD, double kS, double kV) {
        steerPIDConfig.withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV);
        tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerPIDConfig), steerPIDNotSetAlert);
    }
}
