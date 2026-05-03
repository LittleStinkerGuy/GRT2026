package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ComponentStatus.MotorControlMode;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;

public class HoodSubsystem extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private MotorControlMode commandedControlMode = MotorControlMode.Disabled;
    private Angle commandedPositionSetpoint = Rotations.of(0.0);

    private final SysIdRoutine sysIdRoutine;

    @AutoLogOutput(key = "Hood/Mechanism2D")
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("Hood", 0.5, 0.2);
    private final LoggedMechanismLigament2d hoodMech =
        mechanismRoot.append(new LoggedMechanismLigament2d(
            "HoodArm",
            0.4,
            0,
            6.0,
            new Color8Bit(Color.kCornflowerBlue)));

    public HoodSubsystem(HoodIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Hood/kP", pid.kP());
        kI = new LoggedTunableNumber("Hood/kI", pid.kI());
        kD = new LoggedTunableNumber("Hood/kD", pid.kD());
        kS = new LoggedTunableNumber("Hood/kS", pid.kS());
        kV = new LoggedTunableNumber("Hood/kV", pid.kV());
        kA = new LoggedTunableNumber("Hood/kA", pid.kA());

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(1),
                Seconds.of(5),
                (state) -> Logger.recordOutput("Hood/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this));
    }

    public void setDutyCycle(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        io.setDutyCycleOut(speed);
        commandedControlMode = MotorControlMode.DutyCycle;
        Logger.recordOutput("Hood/DutyCycleSetpoint", speed);
    }

    public void setVoltage(Voltage volts) {
        volts = Volts.of(MathUtil.clamp(volts.in(Volts), -12.0, 12.0));
        io.setVoltageOut(volts);
        commandedControlMode = MotorControlMode.Voltage;
        Logger.recordOutput("Hood/VoltageSetpoint", volts);
    }

    public void setPosition(Angle position) {
        position = Rotations.of(MathUtil.clamp(
            position.in(Rotations),
            ShooterConstants.Hood.LOWER_ANGLE_LIMIT.in(Rotations),
            ShooterConstants.Hood.UPPER_ANGLE_LIMIT.in(Rotations)));

        io.setPositionOut(position);
        commandedControlMode = MotorControlMode.Position;
        commandedPositionSetpoint = position;
        Logger.recordOutput("Hood/PositionSetpoint", position);
    }

    public void stop() {
        io.stop();
        commandedControlMode = MotorControlMode.Disabled;
    }

    public Optional<Boolean> atPositionSetpoint() {
        if (commandedControlMode != MotorControlMode.Position) {
            return Optional.empty();
        }
        return Optional.of(commandedPositionSetpoint.isNear(inputs.position, ShooterConstants.Hood.ANGLE_TOLERANCE));
    }

    public Angle getPosition() {
        return inputs.position;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        Logger.recordOutput("Hood/controlMode", commandedControlMode);
        Logger.recordOutput("Hood/atPositionSetpoint", atPositionSetpoint().orElse(false));

        hoodMech.setAngle(inputs.position.in(Degrees));

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> io.updatePID(values[0], values[1], values[2], values[3], values[4], values[5]),
            kP, kI, kD, kS, kV, kA);
    }

    public Command runSysID() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command setHoodManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble()))
            .finallyDo(this::stop);
    }

    public Command goToPosition(Angle position) {
        return this.runOnce(() -> setPosition(position));
    }

    public Command holdPosition(Angle position) {
        return this.startEnd(
            () -> setPosition(position),
            this::stop);
    }

    public Command hideHood() {
        return this.runOnce(() -> setPosition(ShooterConstants.Hood.LOWER_ANGLE_LIMIT))
            .andThen(Commands.waitUntil(() -> atPositionSetpoint().orElse(false)));
    }

    public Command holdDownHood() {
        return this.run(() -> setPosition(ShooterConstants.Hood.LOWER_ANGLE_LIMIT));
    }

    public Command stopHood() {
        return this.runOnce(this::stop);
    }
}
