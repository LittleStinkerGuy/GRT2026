package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SmashAndShootConstants;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import static edu.wpi.first.units.Units.Volts;
import java.util.EnumSet;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class flywheel extends SubsystemBase {
    private NetworkTable NTtable;

    private final LoggedTalon upperMotor;
    private final LoggedTalon secondMotor;
    private VelocityTorqueCurrentFOC spinner = new VelocityTorqueCurrentFOC(0);
    private DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private VoltageOut voltOut = new VoltageOut(0);
    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private Slot0Configs pidSlots = new Slot0Configs();

    private double wantedVe = 0;
    private double targetRPS = 0;

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
        yoTuneThis("Pids/P", val -> pidSlots.withKP(val), ShooterConstants.Flywheel.KP);
        yoTuneThis("Pids/I", val -> pidSlots.withKI(val), ShooterConstants.Flywheel.KI);
        yoTuneThis("Pids/D", val -> pidSlots.withKD(val), ShooterConstants.Flywheel.KD);
        yoTuneThis("Pids/S", val -> pidSlots.withKS(val), ShooterConstants.Flywheel.KS);
        yoTuneThis("Pids/V", val -> pidSlots.withKV(val), ShooterConstants.Flywheel.KV);
        yoTuneThis("setDutyCyclePercent", val -> upperMotor.setControl(new DutyCycleOut(val)), 0);
        yoTuneThis("setMMVTCF", val -> targetRPS = val, 0);

        yoTuneThis("MMAccel", val -> cfg.MotionMagic.MotionMagicAcceleration = val, ShooterConstants.Flywheel.MM_ACCEL);
        yoTuneThis("MMJerk", val -> cfg.MotionMagic.MotionMagicJerk = val, ShooterConstants.Flywheel.MM_JERK);
        yoTuneThis("MMMaxVelo", val -> cfg.MotionMagic.MotionMagicCruiseVelocity = val, ShooterConstants.Flywheel.MM_CRUISE_VELOCITY);

        yoTuneThis("GearReduction", val -> cfg.Feedback.SensorToMechanismRatio = val, ShooterConstants.Flywheel.GEAR_RATIO);
        yoTuneThis("printThisYo", val -> System.out.println("printed this yo: " + val), 0);
    }

    public flywheel(CANBus cn) {
        upperMotor = new LoggedTalon(ShooterConstants.Flywheel.UPPER_MOTOR_ID, cn);
        secondMotor = new LoggedTalon(ShooterConstants.Flywheel.SECOND_MOTOR_ID, cn);
        config();
        configThruNT();
    }

    public void config() {
        cfg.MotorOutput.Inverted = ShooterConstants.Flywheel.F_INVERTED_VALUE;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.Flywheel.MM_CRUISE_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.Flywheel.MM_ACCEL;
        cfg.MotionMagic.MotionMagicJerk = ShooterConstants.Flywheel.MM_JERK;

        pidSlots.withKP(ShooterConstants.Flywheel.KP);
        pidSlots.withKI(ShooterConstants.Flywheel.KI);
        pidSlots.withKD(ShooterConstants.Flywheel.KD);
        pidSlots.withKS(ShooterConstants.Flywheel.KS);
        pidSlots.withKV(ShooterConstants.Flywheel.KV);
        cfg.withSlot0(pidSlots);

        cfg.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.GEAR_RATIO;

        upperMotor.getConfigurator().apply(cfg);
        secondMotor.getConfigurator().apply(cfg);

        secondMotor.setControl(new Follower(ShooterConstants.Flywheel.UPPER_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    public void shoot(double rps) {
        // wantedVe = rps;
        upperMotor.setControl(spinner.withVelocity(targetRPS));
    }

    public double getRPS() {
        return upperMotor.getVelocity().getValueAsDouble();
    }

    public boolean wantedVel() {
        return wantedVe > 0 && Math.abs(wantedVe - upperMotor.getVelocity().getValueAsDouble()) < ShooterConstants.Flywheel.VELOCITY_TOLERANCE_RPS;
    }

    public void dontShoot() {
        wantedVe = 0;
        upperMotor.setControl(spinner.withVelocity(0));
    }

    double commandedDutyCycle = 0;

    public void flySpeed(double speed) {
        if (speed > 0.1) {
            // commandedDutyCycle = 0.6;
            upperMotor.setControl(spinner.withVelocity(targetRPS));
        } else {
            wantedVe = 0;
            upperMotor.setControl(spinner.withVelocity(0));
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Flywheel/FlywheelVelo:", upperMotor.getVelocity(false).getValueAsDouble());
        upperMotor.updateDashboard();
        secondMotor.updateDashboard();
        // sendData();
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
