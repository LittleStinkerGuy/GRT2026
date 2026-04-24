package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HopperConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO {
    private final TalonFX motor;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<AngularAcceleration> accel;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temp;
    private final StatusSignal<Boolean> tempFault;

    public HopperIOTalonFX(int id, LoggedCanivore canivore) {
        motor = new TalonFX(id, canivore);

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
        config.withSlot0(new Slot0Configs()
            .withKP(HopperConstants.kP)
            .withKI(HopperConstants.kI)
            .withKD(HopperConstants.kD)
            .withKS(HopperConstants.kS)
            .withKV(HopperConstants.kV)
            .withKV(HopperConstants.kA));

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

        tryUntilOk(
            5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                accel,
                appliedVoltage,
                supplyCurrent,
                statorCurrent,
                torqueCurrent,
                temp,
                tempFault));
        tryUntilOk(5, () -> motor.optimizeBusUtilization(0, 1.0));

        PhoenixUtil.registerSignals(canivore.getCanType(), position,
            velocity,
            accel,
            appliedVoltage,
            supplyCurrent,
            statorCurrent,
            torqueCurrent,
            temp,
            tempFault);
    }
}
