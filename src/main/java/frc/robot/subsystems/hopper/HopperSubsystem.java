package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;

import java.util.EnumSet;
import java.util.function.Consumer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperConstants.HOPPER_INTAKE;
import frc.robot.util.LoggedTalon;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

public class HopperSubsystem extends SubsystemBase {

    private final LoggedTalon krakenMotor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0); // .withEnableFOC(true); enable if re-run with FOC
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final VoltageOut sysIdVoltage = new VoltageOut(0).withEnableFOC(true);
    private final SysIdRoutine sysIdRoutine;
    private NetworkTableInstance NTinst;
    private NetworkTable NTtable;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private Slot0Configs pidSlots = new Slot0Configs();

    public HopperSubsystem(CANBus canBus) {
        krakenMotor = new LoggedTalon(HopperConstants.KRAKEN_CAN_ID, canBus);
        configureMotor();
        // configThruNT();

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds), // ramp rate: 1 V/s
                Volts.of(7), // step voltage
                Seconds.of(10) // timeout
            ),
            new SysIdRoutine.Mechanism(
                voltage -> krakenMotor.setControl(sysIdVoltage.withOutput(voltage)),
                log -> {
                    log.motor("hopper")
                        .voltage(krakenMotor.getMotorVoltage().getValue())
                        .angularPosition(krakenMotor.getPosition().getValue())
                        .angularVelocity(krakenMotor.getVelocity().getValue());
                },
                this));
    }

    /**
     * @param valueName The name of the value in NetworkTables (ex: "P", "I", "D").
     * @param configSetter A Consumer that takes the new double value and applies it to the s.withKP(value)).
     * @param defaultVal The default value to publish to NetworkTables on startup.
     */
    private void yoTuneThis(String valueName, Consumer<Double> configSetter, double defaultVal) {
        NTtable.getEntry(valueName).setDouble(defaultVal);
        NTtable.addListener(valueName, EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
            configSetter.accept(event.valueData.value.getDouble());

            config.withSlot0(pidSlots);
            krakenMotor.getConfigurator().apply(config);
            System.out.println("Updated: " + valueName + " to this: " + event.valueData.value.getDouble() + "!");
        });
    }

    private void configThruNT() {
        NTinst = NetworkTableInstance.getDefault();
        NTtable = NTinst.getTable("tower");
        yoTuneThis("Pids/P", val -> pidSlots.withKP(val), HopperConstants.KP);
        yoTuneThis("Pids/I", val -> pidSlots.withKI(val), HopperConstants.KI);
        yoTuneThis("Pids/D", val -> pidSlots.withKD(val), HopperConstants.KD);
        yoTuneThis("Pids/S", val -> pidSlots.withKS(val), HopperConstants.KS);
        yoTuneThis("Pids/V", val -> pidSlots.withKV(val), HopperConstants.KV);
        // tuneThis("A", val -> pidSlots.withKP(val), TowerConstants.KA);
        // tuneThis("G", val -> pidSlots.withKP(val), TowerConstants.KG);
        yoTuneThis("setDutyCyclePercent", val -> krakenMotor.setControl(dutyCycleControl.withOutput(val)), 0);
        yoTuneThis("setMMVTCF", val -> krakenMotor.setControl(new VelocityVoltage(val)), 0);

        yoTuneThis("MMAccel", val -> config.MotionMagic.MotionMagicAcceleration = val, 100);
        yoTuneThis("MMJerk", val -> config.MotionMagic.MotionMagicJerk = val, 1000);
        yoTuneThis("MMMaxVelo", val -> config.MotionMagic.MotionMagicCruiseVelocity = val, 100);

        yoTuneThis("GearReduction", val -> config.Feedback.SensorToMechanismRatio = val,
            HopperConstants.GEAR_REDUCTION);
        yoTuneThis("printThisYo", val -> System.out.println("printed this yo: " + val), 0);
    }

    private void configureMotor() {
        // Motor output
        config.withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(HopperConstants.HOPPERINVERTED));

        // Current limits
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(HopperConstants.STATOR_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(HopperConstants.STATOR_CURRENT_LIMIT_AMPS)));
        config.Feedback.SensorToMechanismRatio = HopperConstants.GEAR_REDUCTION;
        // Velocity control PID (Slot 0)
        config.withSlot0(new Slot0Configs()
            .withKP(HopperConstants.KP)
            .withKI(HopperConstants.KI)
            .withKD(HopperConstants.KD)
            .withKS(HopperConstants.KS)
            .withKV(HopperConstants.KV)
            .withKV(HopperConstants.KA));

        krakenMotor.getConfigurator().apply(config);
    }

    public void setHopper(HOPPER_INTAKE state) {
        switch (state) {
            case BALLIN:
                krakenMotor.setControl(velocityControl.withVelocity(HopperConstants.TARGET_RPS));
                break;
            case BALLOUT:
                krakenMotor.setControl(velocityControl.withVelocity(-HopperConstants.TARGET_RPS));
                break;
            case STOP:
                krakenMotor.stopMotor();
                break;
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void setManualControl(double percentOutput) {
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));
        krakenMotor.setControl(dutyCycleControl.withOutput(percentOutput));
    }

    @Override
    public void periodic() {
        krakenMotor.updateDashboard();

        SmartDashboard.putNumber("sysIDTest/hopperSetpoint(RPS)", krakenMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("sysIDTest/hopperVelo(RPS)", krakenMotor.getVelocity(false).getValueAsDouble());
    }
}
