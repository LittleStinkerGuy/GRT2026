package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperConstants.HopperIntake;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;
import frc.robot.util.ComponentStatus.MotorControlMode;

public class HopperSubsystem extends SubsystemBase {
    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber motionMagicAccel =
        new LoggedTunableNumber("Hopper/motionMagicAccel_rotPerSec2",
            HopperConstants.MM_ACCEL.in(RotationsPerSecondPerSecond));
    private final LoggedTunableNumber motionMagicVelo =
        new LoggedTunableNumber("Hopper/motionMagicVelocity_rotPerSec",
            HopperConstants.MM_MAX_VELO.in(RotationsPerSecond));
    private final LoggedTunableNumber motionMagicJerk =
        new LoggedTunableNumber("Hopper/motionMagicJerk_rotPerSec3",
            HopperConstants.MM_JERK.in(RotationsPerSecondPerSecond.per(Second)));

    private MotorControlMode commandedControlMode = MotorControlMode.Disabled;
    private AngularVelocity commandedVelocitySetpoint = RotationsPerSecond.of(0.0);

    private final SysIdRoutine sysIdRoutine;

    private static final int HOPPER_VANES = 4;
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("HopperSpinner", 0.5, 0.5);
    private final LoggedMechanismLigament2d[] vaneLigaments = new LoggedMechanismLigament2d[HOPPER_VANES];

    public HopperSubsystem(HopperIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Hopper/kP", pid.kP());
        kI = new LoggedTunableNumber("Hopper/kI", pid.kI());
        kD = new LoggedTunableNumber("Hopper/kD", pid.kD());
        kS = new LoggedTunableNumber("Hopper/kS", pid.kS());
        kV = new LoggedTunableNumber("Hopper/kV", pid.kV());
        kA = new LoggedTunableNumber("Hopper/kA", pid.kA());

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());

        for (int i = 0; i < HOPPER_VANES; i++) {
            vaneLigaments[i] = mechanismRoot.append(
                new LoggedMechanismLigament2d(
                    "Vane" + i,
                    0.4,
                    i * (360.0 / HOPPER_VANES),
                    6.0,
                    new Color8Bit(Color.kBlueViolet)));
        }

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds), // ramp rate: 1 V/s
                Volts.of(7), // step voltage
                Seconds.of(10), // timeout
                (state) -> Logger.recordOutput("Hopper/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> setVoltage(voltage),
                null,
                this));
    }

    public void setDutyCycle(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        io.setDutyCycleOut(speed);
        commandedControlMode = MotorControlMode.DutyCycle;
        Logger.recordOutput("Hopper/DutyCycleSetpoint", speed);
    }

    public void setVoltage(Voltage volts) {
        volts = Volts.of(MathUtil.clamp(volts.in(Volts), -12.0, 12.0));
        io.setVoltageOut(volts);
        commandedControlMode = MotorControlMode.Voltage;
        Logger.recordOutput("Hopper/VoltageSetpoint", volts);
    }

    public void setVelocity(AngularVelocity velo) {
        io.setVelocityOut(velo);
        commandedControlMode = MotorControlMode.Velocity;
        commandedVelocitySetpoint = velo;
        Logger.recordOutput("Hopper/VelocitySetpoint", velo);
    }

    public void stop() {
        io.stop();
        commandedControlMode = MotorControlMode.Disabled;
    }

    public Optional<Boolean> atSetpoint() {
        if (commandedControlMode != MotorControlMode.Velocity) {
            return Optional.empty();
        }
        return Optional.of(commandedVelocitySetpoint.isNear(inputs.velocity, HopperConstants.VELOCITY_TOLERANCE));
    }

    public void setHopperState(HopperIntake state) {
        switch (state) {
            case BALL_IN:
                setVelocity(HopperConstants.TARGET_RPS);
                break;
            case BALL_OUT:
                setVelocity(HopperConstants.TARGET_RPS.unaryMinus());
                break;
            default:
                stop();
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);

        Logger.recordOutput("Hopper/controlMode", commandedControlMode);
        Logger.recordOutput("Hopper/atVelocitySetpoint", atSetpoint().orElse(false));

        double spinnerDeg = inputs.position.in(Degrees);
        for (int i = 0; i < HOPPER_VANES; i++) {
            vaneLigaments[i].setAngle(spinnerDeg + i * (360.0 / HOPPER_VANES));
        }
        Logger.recordOutput("Hopper/Mechanism2d", mechanism);

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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command fullSysID() {
        return sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdDynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command setHopperManualSpeed(DoubleSupplier speedSupplier) {
        return run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(interrupted -> stop());
    }

    public Command runHopperState(HopperIntake state) {
        return this.runEnd(
            () -> {
                setHopperState(state);
            },
            this::stop);
    }

    public Command runHopperOut() {
        return runHopperState(HopperIntake.BALL_OUT);
    }

    public Command runHopperIn() {
        return runHopperState(HopperIntake.BALL_IN);
    }

    public Command stopHopper() {
        return this.runOnce(this::stop);
    }
}
