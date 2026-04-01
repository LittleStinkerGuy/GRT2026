package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import java.util.EnumSet;
import java.util.function.Consumer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.TowerConstants.TOWER_INTAKE;
import frc.robot.util.LoggedTalon;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

public class towerRollers extends SubsystemBase {

    private final LoggedTalon krakenMotor;
    private final MotionMagicVelocityTorqueCurrentFOC velocityControl;
    private final DutyCycleOut dutyCycleControl;
    private final VoltageOut sysIdVoltage = new VoltageOut(0);
    private final SysIdRoutine sysIdRoutine;
    private NetworkTable NTtable;
    private DoubleSubscriber sub;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private Slot0Configs pidSlots = new Slot0Configs();

    public towerRollers(CANBus canBus) {
        krakenMotor = new LoggedTalon(TowerConstants.KRAKEN_CAN_ID, canBus);
        velocityControl = new MotionMagicVelocityTorqueCurrentFOC(0);
        dutyCycleControl = new DutyCycleOut(0);
        configureMotor();
        configThruNT();

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds), // ramp rate: 1 V/s
                Volts.of(7), // step voltage
                Seconds.of(10) // timeout
            ),
            new SysIdRoutine.Mechanism(
                voltage -> krakenMotor.setControl(sysIdVoltage.withOutput(voltage)),
                log -> {
                    log.motor("towerRollers")
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
        // configSetter.accept(defaultVal);
        // config.withSlot0(pidSlots);
        krakenMotor.getConfigurator().apply(config);

        NTtable.getEntry(valueName).setDouble(defaultVal);
        NTtable.addListener(valueName, EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
            configSetter.accept(event.valueData.value.getDouble());

            config.withSlot0(pidSlots);
            krakenMotor.getConfigurator().apply(config);
            System.out.println("Updated: " + valueName + " to this: " + event.valueData.value.getDouble() + "!");
        });
    }

    private void configThruNT() {
        NTtable = NetworkTableInstance.getDefault().getTable("tuneTower");
        yoTuneThis("Pids/P", val -> pidSlots.withKP(val), TowerConstants.KP);
        yoTuneThis("Pids/I", val -> pidSlots.withKI(val), TowerConstants.KI);
        yoTuneThis("Pids/D", val -> pidSlots.withKD(val), TowerConstants.KD);
        yoTuneThis("Pids/S", val -> pidSlots.withKS(val), TowerConstants.KS);
        yoTuneThis("Pids/V", val -> pidSlots.withKV(val), TowerConstants.KV);
        // tuneThis("A", val -> pidSlots.withKP(val), TowerConstants.KA);
        // tuneThis("G", val -> pidSlots.withKP(val), TowerConstants.KG);
        yoTuneThis("setDutyCyclePercent", val -> krakenMotor.setControl(new DutyCycleOut(val)), 0);
        yoTuneThis("setMMVTCF", val -> krakenMotor.setControl(new VelocityVoltage(val)), 0);

        yoTuneThis("MMAccel", val -> config.MotionMagic.MotionMagicAcceleration = val, TowerConstants.MM_ACCEL);
        yoTuneThis("MMJerk", val -> config.MotionMagic.MotionMagicJerk = val, TowerConstants.MM_JERK);
        yoTuneThis("MMMaxVelo", val -> config.MotionMagic.MotionMagicCruiseVelocity = val, TowerConstants.MM_MAXVELO);

        yoTuneThis("GearReduction", val -> config.Feedback.SensorToMechanismRatio = val, TowerConstants.GEAR_REDUCTION);
        yoTuneThis("printThisYo", val -> System.out.println("printed this yo: " + val), 0);
    }

    private void configureMotor() {
        // Motor output
        config.withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(TowerConstants.HOPPERINVERTED));

        // Current limits
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(TowerConstants.STATOR_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(TowerConstants.STATOR_CURRENT_LIMIT_AMPS)));
        config.MotionMagic.MotionMagicCruiseVelocity = TowerConstants.MM_MAXVELO; // rotations/sec^2
        config.MotionMagic.MotionMagicAcceleration = TowerConstants.MM_ACCEL; // rotations/sec^2
        config.MotionMagic.MotionMagicJerk = TowerConstants.MM_JERK; // optional
        config.Feedback.SensorToMechanismRatio = TowerConstants.GEAR_REDUCTION;
        // Velocity control PID (Slot 0)
        pidSlots.withKP(TowerConstants.KP);
        pidSlots.withKI(TowerConstants.KI);
        pidSlots.withKD(TowerConstants.KD);
        pidSlots.withKS(TowerConstants.KS);
        pidSlots.withKV(TowerConstants.KV);
        config.withSlot0(pidSlots);

        krakenMotor.getConfigurator().apply(config);
    }

    public void setTower(TOWER_INTAKE state) {
        switch (state) {
            case BALLUP:
                krakenMotor.setControl(new MotionMagicVelocityTorqueCurrentFOC(TowerConstants.TARGET_RPS));
                break;
            case BALLDOWN:
                krakenMotor.setControl(new MotionMagicVelocityTorqueCurrentFOC(-TowerConstants.TARGET_RPS));
                break;
            case STOP:
                krakenMotor.setControl(new MotionMagicVelocityTorqueCurrentFOC(0.0));
                break;
        }
    }

    public boolean correctRoll() {
        if (Math.abs(TowerConstants.TARGET_RPS - krakenMotor.getVelocity().getValueAsDouble()) < 2) {
            return true;
        } else {
            return false;
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
    }
}
