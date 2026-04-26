package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import java.util.ArrayList;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.HopperConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO {
    private final TalonFX motor;
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final Slot0Configs pidConfig;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);

    private final Alert failedToSetFrequencyAlert = new Alert("Hopper", "Failed to set status signal frequency!", AlertType.kError);

    ArrayList<BaseStatusSignal> signals;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<AngularAcceleration> accel;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temp;
    private final StatusSignal<Boolean> tempFault;

    public HopperIOTalonFX(LoggedCanivore canivore) {
        motor = new TalonFX(HopperConstants.KRAKEN_CAN_ID, canivore);

        // Motor output
        config.withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(HopperConstants.HOPPER_INVERTED));

        // Current limits
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(HopperConstants.STATOR_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(HopperConstants.STATOR_CURRENT_LIMIT_AMPS)));
        config.Feedback.SensorToMechanismRatio = HopperConstants.GEAR_REDUCTION;

        // Velocity control PID (Slot 0)
        pidConfig = new Slot0Configs()
            .withKP(HopperConstants.kP)
            .withKI(HopperConstants.kI)
            .withKD(HopperConstants.kD)
            .withKS(HopperConstants.kS)
            .withKV(HopperConstants.kV)
            .withKA(HopperConstants.kA);
        config.withSlot0(pidConfig);

        tryUntilOk(5, () -> motor.getConfigurator().apply(config));

        position = motor.getPosition(false);
        velocity = motor.getVelocity(false);
        accel = motor.getAcceleration(false);
        appliedVoltage = motor.getMotorVoltage(false);
        supplyCurrent = motor.getSupplyCurrent(false);
        statorCurrent = motor.getStatorCurrent(false);
        torqueCurrent = motor.getTorqueCurrent(false);
        temp = motor.getDeviceTemp(false);
        tempFault = motor.getFault_DeviceTemp(false);

        signals = new ArrayList<>();
        signals.add(position);
        signals.add(velocity);
        signals.add(accel);
        signals.add(appliedVoltage);
        signals.add(supplyCurrent);
        signals.add(statorCurrent);
        signals.add(torqueCurrent);
        signals.add(temp);
        signals.add(tempFault);

        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals), failedToSetFrequencyAlert);
        tryUntilOk(5, () -> motor.optimizeBusUtilization(0, 1.0));
        PhoenixUtil.registerSignals(canivore.getCanType(), signals);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.acceleration = accel.getValue();
        inputs.appliedVoltage = appliedVoltage.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.temp = temp.getValue();
        inputs.tempFault = tempFault.getValue();
        inputs.connected = BaseStatusSignal.isAllGood(signals);
    }

    @Override
    public void updatePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        pidConfig.withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV).withKA(kA);
        System.out.println("changes pid");
        tryUntilOk(5, () -> motor.getConfigurator().apply(pidConfig));
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        dutyCycle = Math.max(-1.0, Math.min(1.0, dutyCycle));
        motor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
