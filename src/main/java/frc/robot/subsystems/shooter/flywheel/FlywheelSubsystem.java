package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ComponentStatus.MotorControlMode;
import frc.robot.util.LoggedSetpointTracker;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;
import frc.robot.util.RollerMechanism2D;

public class FlywheelSubsystem extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber motionMagicAccel =
        new LoggedTunableNumber("Flywheel/motionMagicAccel_rotPerSec2", ShooterConstants.Flywheel.MM_ACCEL_RPS2);
    private final LoggedTunableNumber motionMagicVelo =
        new LoggedTunableNumber("Flywheel/motionMagicVelocity_rotPerSec", ShooterConstants.Flywheel.MM_MAX_VELO_RPS);
    private final LoggedTunableNumber motionMagicJerk =
        new LoggedTunableNumber("Flywheel/motionMagicJerk_rotPerSec3", ShooterConstants.Flywheel.MM_JERK_RPS3);

    private final LoggedSetpointTracker setpointTracker = new LoggedSetpointTracker(
        "Flywheel",
        MotorControlMode.DutyCycle,
        MotorControlMode.Voltage,
        MotorControlMode.Velocity);
    private double commandedVoltageSetpoint = 0.0;
    private double commandedVelocitySetpoint = 0.0;

    private final SysIdRoutine sysIdRoutine;

    private final RollerMechanism2D mechanism = new RollerMechanism2D(0.4);
    @AutoLogOutput(key = "Flywheel/Mechanism2D")
    private final LoggedMechanism2d mechanism2d = mechanism.getMechanism2d();

    public FlywheelSubsystem(FlywheelIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Flywheel/kP", pid.kP());
        kI = new LoggedTunableNumber("Flywheel/kI", pid.kI());
        kD = new LoggedTunableNumber("Flywheel/kD", pid.kD());
        kS = new LoggedTunableNumber("Flywheel/kS", pid.kS());
        kV = new LoggedTunableNumber("Flywheel/kV", pid.kV());
        kA = new LoggedTunableNumber("Flywheel/kA", pid.kA());

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds),
                Volts.of(7),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Flywheel/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this));
    }

    public void setDutyCycle(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        io.setDutyCycleOut(speed);
        setpointTracker.updateSetpoint(speed, MotorControlMode.DutyCycle);
    }

    public void setVoltage(double volts) {
        commandedVoltageSetpoint = MathUtil.clamp(volts, -12.0, 12.0);
        io.setVoltageOut(commandedVoltageSetpoint);
        setpointTracker.updateSetpoint(commandedVoltageSetpoint, MotorControlMode.Voltage);
    }

    private void setVoltage(Voltage volts) {
        setVoltage(volts.in(Volts));
    }

    public void setVelocity(double velocityRPS) {
        io.setVelocityOut(velocityRPS);
        commandedVelocitySetpoint = velocityRPS;
        setpointTracker.updateSetpoint(commandedVelocitySetpoint, MotorControlMode.Velocity);
    }

    public void stop() {
        io.stop();
        setpointTracker.setControlMode(MotorControlMode.Disabled);
    }

    public Optional<Boolean> atSetpoint() {
        if (setpointTracker.getControlMode() != MotorControlMode.Velocity) {
            return Optional.empty();
        }
        return Optional.of(Math.abs(commandedVelocitySetpoint - inputs.velocityRPS) <= ShooterConstants.Flywheel.VELOCITY_TOLERANCE_RPS);
    }

    public double getVelocity() {
        return inputs.velocityRPS;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        if (DriverStation.isDisabled()) {
            setpointTracker.setControlMode(MotorControlMode.Disabled);
        }

        setpointTracker.logAll();
        Logger.recordOutput("Flywheel/atVelocitySetpoint", atSetpoint().orElse(false));

        mechanism.setPosition(inputs.positionRot);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> io.updatePID(values[0], values[1], values[2], values[3], values[4], values[5]),
            kP, kI, kD, kS, kV, kA);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> io.updateMotionMagicConfig(values[0], values[1], values[2]),
            motionMagicAccel, motionMagicVelo, motionMagicJerk);

        LoggedTracer.record("Flywheel");
    }

    public Command runSysID() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command setFlywheelManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command rampToVelocity(DoubleSupplier rpsSupplier) {
        return this.run(() -> setVelocity(rpsSupplier.getAsDouble()))
            .finallyDo(this::stop);
    }

    public Command stopFlywheel() {
        return this.runOnce(this::stop);
    }
}
