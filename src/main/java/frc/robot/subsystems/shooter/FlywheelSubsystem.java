package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.util.ComponentStatus.MotorControlMode;
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
        new LoggedTunableNumber("Flywheel/motionMagicAccel_rotPerSec2",
            ShooterConstants.Flywheel.MM_ACCEL.in(RotationsPerSecondPerSecond));
    private final LoggedTunableNumber motionMagicVelo =
        new LoggedTunableNumber("Flywheel/motionMagicVelocity_rotPerSec",
            ShooterConstants.Flywheel.MM_MAX_VELO.in(RotationsPerSecond));
    private final LoggedTunableNumber motionMagicJerk =
        new LoggedTunableNumber("Flywheel/motionMagicJerk_rotPerSec3",
            ShooterConstants.Flywheel.MM_JERK.in(RotationsPerSecondPerSecond.per(Second)));

    private MotorControlMode commandedControlMode = MotorControlMode.Disabled;
    private AngularVelocity commandedVelocitySetpoint = RotationsPerSecond.of(0.0);

    private final SysIdRoutine sysIdRoutine;

    private final RollerMechanism2D mechanism = new RollerMechanism2D(0.4);

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
        commandedControlMode = MotorControlMode.DutyCycle;
        Logger.recordOutput("Flywheel/DutyCycleSetpoint", speed);
    }

    public void setVoltage(Voltage volts) {
        volts = Volts.of(MathUtil.clamp(volts.in(Volts), -12.0, 12.0));
        io.setVoltageOut(volts);
        commandedControlMode = MotorControlMode.Voltage;
        Logger.recordOutput("Flywheel/VoltageSetpoint", volts);
    }

    public void setVelocity(AngularVelocity velo) {
        io.setVelocityOut(velo);
        commandedControlMode = MotorControlMode.Velocity;
        commandedVelocitySetpoint = velo;
        Logger.recordOutput("Flywheel/VelocitySetpoint", velo);
    }

    public void stop() {
        io.stop();
        commandedControlMode = MotorControlMode.Disabled;
    }

    public Optional<Boolean> atSetpoint() {
        if (commandedControlMode != MotorControlMode.Velocity) {
            return Optional.empty();
        }
        return Optional.of(commandedVelocitySetpoint.isNear(inputs.velocity, ShooterConstants.Flywheel.VELOCITY_TOLERANCE));
    }

    public AngularVelocity getVelocity() {
        return inputs.velocity;
    }

    // ---- Backwards-compat shims for existing commands ---------------------------------

    /** @deprecated use {@link #setVelocity(AngularVelocity)} */
    @Deprecated
    public void shoot(double rps) {
        setVelocity(RotationsPerSecond.of(rps));
    }

    /** @deprecated use {@link #stop()} */
    @Deprecated
    public void dontShoot() {
        stop();
    }

    /** @deprecated use {@link #setVelocity(AngularVelocity)} or {@link #stop()} directly */
    @Deprecated
    public void flySpeed(double speed) {
        if (speed > 0.1) {
            setVelocity(RotationsPerSecond.of(ShooterConstants.Flywheel.FLYWHEEL_MAX_SPEED));
        } else {
            stop();
        }
    }

    /** @deprecated use {@link #atSetpoint()} */
    @Deprecated
    public boolean wantedVel() {
        return atSetpoint().orElse(false);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        Logger.recordOutput("Flywheel/controlMode", commandedControlMode);
        Logger.recordOutput("Flywheel/atVelocitySetpoint", atSetpoint().orElse(false));

        mechanism.setPosition(inputs.position);
        Logger.recordOutput("Flywheel/Mechanism2D", mechanism.getMechanism2d());

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> io.updatePID(values[0], values[1], values[2], values[3], values[4], values[5]),
            kP, kI, kD, kS, kV, kA);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> io.updateMotionMagicConfig(
                RotationsPerSecondPerSecond.of(values[0]),
                RotationsPerSecond.of(values[1]),
                RotationsPerSecondPerSecond.of(values[2]).per(Second)),
            motionMagicAccel, motionMagicVelo, motionMagicJerk);
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
        return this.run(() -> setVelocity(RotationsPerSecond.of(rpsSupplier.getAsDouble())))
            .finallyDo(this::stop);
    }

    public Command stopFlywheel() {
        return this.runOnce(this::stop);
    }
}
