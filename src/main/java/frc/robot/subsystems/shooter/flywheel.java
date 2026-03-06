package frc.robot.subsystems.shooter;

import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.railgunConstants;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import java.util.EnumSet;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class flywheel extends SubsystemBase {
    private NetworkTable NTtable;

    private final LoggedTalon upperMotor;
    private final LoggedTalon secondMotor;
    private MotionMagicVelocityVoltage spinner = new MotionMagicVelocityVoltage(0);
    private DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private TalonFXConfiguration cfg;
    private Slot0Configs pidSlots = new Slot0Configs();

    private double wantedVe = 0;

    private static final String LOG_PREFIX = "FlyWheel/";

    private void yoTuneThis(String valueName, Consumer<Double> configSetter, double defaultVal) {
        NTtable.getEntry(valueName).setDouble(defaultVal);
        NTtable.addListener(valueName, EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
            configSetter.accept(event.valueData.value.getDouble());

            cfg.withSlot0(pidSlots);
            upperMotor.getConfigurator().apply(cfg);
            secondMotor.getConfigurator().apply(cfg);

            System.out.println("Updated: " + valueName + " to this: " + event.valueData.value.getDouble() + "!");
        });
    }

    private void configThruNT() {
        NTtable = NetworkTableInstance.getDefault().getTable("tuneFlywheel");
        yoTuneThis("Pids/P", val -> pidSlots.withKP(val), railgunConstants.KP);
        yoTuneThis("Pids/I", val -> pidSlots.withKI(val), railgunConstants.KI);
        yoTuneThis("Pids/D", val -> pidSlots.withKD(val), railgunConstants.KD);
        yoTuneThis("Pids/S", val -> pidSlots.withKS(val), railgunConstants.KS);
        yoTuneThis("Pids/V", val -> pidSlots.withKV(val), railgunConstants.KV);
        // tuneThis("A", val -> pidSlots.withKP(val), TowerConstants.KA);
        // tuneThis("G", val -> pidSlots.withKP(val), TowerConstants.KG);
        yoTuneThis("setDutyCyclePercent", val -> upperMotor.setControl(new DutyCycleOut(val)), 0);
        yoTuneThis("setMMVTCF", val -> upperMotor.setControl(new VelocityVoltage(val)), 0);

        yoTuneThis("MMAccel", val -> cfg.MotionMagic.MotionMagicAcceleration = val, TowerConstants.MM_ACCEL);
        yoTuneThis("MMJerk", val -> cfg.MotionMagic.MotionMagicJerk = val, TowerConstants.MM_JERK);
        yoTuneThis("MMMaxVelo", val -> cfg.MotionMagic.MotionMagicCruiseVelocity = val, TowerConstants.MM_MAXVELO);

        yoTuneThis("GearReduction", val -> cfg.Feedback.SensorToMechanismRatio = val, railgunConstants.gearRatioUpper);
        yoTuneThis("printThisYo", val -> System.out.println("printed this yo: " + val), 0);
    }

    public flywheel(CANBus cn) {
        upperMotor = new LoggedTalon(railgunConstants.upperId, cn);
        secondMotor = new LoggedTalon(railgunConstants.secondId, cn);

        config();
        configThruNT();
    }

    public void config() {
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotionMagic.MotionMagicCruiseVelocity = 500; // target RPS cap
        cfg.MotionMagic.MotionMagicAcceleration = 30; // RPS per second
        cfg.MotionMagic.MotionMagicJerk = 150; // optional, smoothness

        pidSlots.withKP(TowerConstants.KP);
        pidSlots.withKP(TowerConstants.KI);
        pidSlots.withKP(TowerConstants.KD);
        pidSlots.withKP(TowerConstants.KS);
        pidSlots.withKP(TowerConstants.KV);
        cfg.withSlot0(pidSlots);

        cfg.Feedback.SensorToMechanismRatio = railgunConstants.gearRatioUpper;

        upperMotor.getConfigurator().apply(cfg);
        secondMotor.getConfigurator().apply(cfg);

        secondMotor.setControl(new Follower(railgunConstants.upperId, MotorAlignmentValue.Opposed));
    }

    public void shoot(double rps) {
        wantedVe = rps;
        upperMotor.setControl(spinner.withVelocity(rps));
    }

    public double getRPS() {
        return upperMotor.getVelocity().getValueAsDouble();
    }

    public boolean wantedVel() {
        if (Math.abs(wantedVe - upperMotor.getVelocity().getValueAsDouble()) < 2) {
            return true;
        } else {
            return false;
        }
    }

    public void dontShoot() {
        wantedVe = 0;
        upperMotor.setControl(spinner.withVelocity(0));
    }

    double commandedDutyCycle = 0;

    public void flySpeed(double speed) {
        if (speed > 0.75) {
            commandedDutyCycle = 0.65;
            upperMotor.setControl(dutyCycl.withOutput(commandedDutyCycle));
        } else {
            commandedDutyCycle = 0.0;
            upperMotor.setControl(dutyCycl.withOutput(0.0));
        }
    }

    @Override
    public void periodic() {
        upperMotor.updateDashboard();
        secondMotor.updateDashboard();
        sendData();
    }

    public void sendData() {
        Logger.recordOutput(LOG_PREFIX + "PositionRotations",
                upperMotor.getPosition().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "VelocityRPS",
                upperMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "AppliedVolts",
                upperMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "SupplyVoltage",
                upperMotor.getSupplyVoltage().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "StatorCurrentAmps",
                upperMotor.getStatorCurrent().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "SupplyCurrentAmps",
                upperMotor.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "TemperatureC",
                upperMotor.getDeviceTemp().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "CommandedDutyCycle",
                commandedDutyCycle);

        Logger.recordOutput(LOG_PREFIX + "Connected",
                upperMotor.isConnected());
    }

}
