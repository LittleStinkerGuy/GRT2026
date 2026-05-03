package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TowerConstants.TowerIntake;
import frc.robot.Constants.TowerConstants;
import frc.robot.util.LoggedTalon;
import java.util.EnumSet;
import java.util.function.Consumer;

public class TowerRollersSubsystem extends SubsystemBase {

    private final LoggedTalon krakenMotor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0); // .withEnableFOC(true); enable if re-run with FOC
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final VoltageOut sysIdVoltage = new VoltageOut(0).withEnableFOC(true);
    private final SysIdRoutine sysIdRoutine;
    private NetworkTable ntTable;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private Slot0Configs pidSlots = new Slot0Configs();

    public TowerRollersSubsystem(CANBus canBus) {
        krakenMotor = new LoggedTalon(TowerConstants.KRAKEN_CAN_ID, canBus);
        configureMotor();
        // configThruNt();

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

        ntTable.getEntry(valueName).setDouble(defaultVal);
        ntTable.addListener(valueName, EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
            configSetter.accept(event.valueData.value.getDouble());

            config.withSlot0(pidSlots);
            krakenMotor.getConfigurator().apply(config);
            System.out.println("Updated: " + valueName + " to this: " + event.valueData.value.getDouble() + "!");
        });
    }

    private void configThruNt() {
        ntTable = NetworkTableInstance.getDefault().getTable("tuneTower");
        yoTuneThis("Pids/P", val -> pidSlots.withKP(val), TowerConstants.kP);
        yoTuneThis("Pids/I", val -> pidSlots.withKI(val), TowerConstants.kI);
        yoTuneThis("Pids/D", val -> pidSlots.withKD(val), TowerConstants.kD);
        yoTuneThis("Pids/S", val -> pidSlots.withKS(val), TowerConstants.kS);
        yoTuneThis("Pids/V", val -> pidSlots.withKV(val), TowerConstants.kV);
        // tuneThis("A", val -> pidSlots.withKP(val), TowerConstants.KA);
        // tuneThis("G", val -> pidSlots.withKP(val), TowerConstants.KG);
        yoTuneThis("setDutyCyclePercent", val -> krakenMotor.setControl(new DutyCycleOut(val)), 0);
        yoTuneThis("setMMVTCF", val -> krakenMotor.setControl(new VelocityVoltage(val)), 0);

        // yoTuneThis("MMAccel", val -> config.MotionMagic.MotionMagicAcceleration = val, TowerConstants.MM_ACCEL);
        // yoTuneThis("MMJerk", val -> config.MotionMagic.MotionMagicJerk = val, TowerConstants.MM_JERK);
        // yoTuneThis("MMMaxVelo", val -> config.MotionMagic.MotionMagicCruiseVelocity = val, TowerConstants.MM_MAX_VELO);

        yoTuneThis("GearReduction", val -> config.Feedback.SensorToMechanismRatio = val, TowerConstants.GEAR_REDUCTION);
        yoTuneThis("printThisYo", val -> System.out.println("printed this yo: " + val), 0);
    }

    private void configureMotor() {
        // Motor output
        config.withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(TowerConstants.HOPPER_INVERTED));

        // Current limits
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(TowerConstants.STATOR_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(TowerConstants.STATOR_CURRENT_LIMIT_AMPS)));

        config.Feedback.SensorToMechanismRatio = TowerConstants.GEAR_REDUCTION;
        // Velocity control PID (Slot 0)
        pidSlots.withKP(TowerConstants.kP);
        pidSlots.withKI(TowerConstants.kI);
        pidSlots.withKD(TowerConstants.kD);
        pidSlots.withKS(TowerConstants.kS);
        pidSlots.withKV(TowerConstants.kV);
        pidSlots.withKA(TowerConstants.kA);
        config.withSlot0(pidSlots);

        krakenMotor.getConfigurator().apply(config);
    }

    public void setTower(TowerIntake state) {
        switch (state) {
            case BALL_UP:
                krakenMotor.setControl(velocityControl.withVelocity(TowerConstants.TARGET_VELO));
                break;
            case BALL_DOWN:
                krakenMotor.setControl(velocityControl.withVelocity(TowerConstants.TARGET_VELO.unaryMinus()));
                break;
            case STOP:
                krakenMotor.stopMotor();
                break;
            default:
                break;
        }
    }

    public boolean correctRoll() {
        double errorRPS = Math.abs(TowerConstants.TARGET_VELO.in(RotationsPerSecond) - krakenMotor.getVelocity().getValueAsDouble());
        if (errorRPS < TowerConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond)) {
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

        SmartDashboard.putNumber("sysIDTest/towerSetpoint(RPS)", krakenMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("sysIDTest/towerVelo(RPS)", krakenMotor.getVelocity(false).getValueAsDouble());
    }
}
