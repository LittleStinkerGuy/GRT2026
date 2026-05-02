package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.ComponentStatus.MotorControlMode;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;

public class PivotSubsystem extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private MotorControlMode commandedControlMode = MotorControlMode.Disabled;
    private double commandedDutyCycleSetpoint = 0.0;
    private Voltage commandedVoltageSetpoint = Volts.of(0.0);
    private Angle commandedPositionSetpoint = Rotations.of(0.0);

    private final SysIdRoutine sysIdRoutine;

    @AutoLogOutput(key = "Pivot/Mechanism2D")
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("Pivot", 0.4, 0.2);
    private final LoggedMechanismLigament2d pivotMech =
        mechanismRoot.append(new LoggedMechanismLigament2d(
            "PivotArm",
            0.5,
            0,
            6.0,
            new Color8Bit(Color.kBlueViolet)));

    public PivotSubsystem(PivotIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Pivot/kP", pid.kP());
        kI = new LoggedTunableNumber("Pivot/kI", pid.kI());
        kD = new LoggedTunableNumber("Pivot/kD", pid.kD());
        kS = new LoggedTunableNumber("Pivot/kS", pid.kS());
        kV = new LoggedTunableNumber("Pivot/kV", pid.kV());
        kA = new LoggedTunableNumber("Pivot/kA", pid.kA());

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds), // ramp rate: 0.5 V/s (slow for limited range)
                Volts.of(1), // step voltage
                Seconds.of(5), // timeout
                (state) -> Logger.recordOutput("Pivot/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this));
    }

    public void setDutyCycle(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        io.setDutyCycleOut(speed);
        commandedControlMode = MotorControlMode.DutyCycle;
        commandedDutyCycleSetpoint = speed;
    }

    public void setVoltage(Voltage volts) {
        volts = Volts.of(MathUtil.clamp(volts.in(Volts), -12, 12));
        io.setVoltageOut(volts);
        commandedControlMode = MotorControlMode.Voltage;
        commandedVoltageSetpoint = volts;
    }

    public void setPosition(Angle position) {
        io.setPositionOut(position);
        commandedControlMode = MotorControlMode.Position;
        commandedPositionSetpoint = position;
    }

    public void stop() {
        io.stop();
        commandedControlMode = MotorControlMode.Disabled;
    }

    public Optional<Boolean> atPositionSetpoint() {
        if (commandedControlMode != MotorControlMode.Position) {
            return Optional.empty();
        }
        return Optional.of(commandedPositionSetpoint.isNear(inputs.position, Degrees.of(5)));
    }

    private void logSetpoints(double dutyCycleSetpoint, Voltage voltageSetpoint, Angle positionSetpoint) {
        Logger.recordOutput("Pivot/DutyCycleSetpoint", dutyCycleSetpoint);
        Logger.recordOutput("Pivot/VoltageSetpoint", voltageSetpoint);
        Logger.recordOutput("Pivot/PositionSetpoint", positionSetpoint);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        Logger.recordOutput("Pivot/controlMode", commandedControlMode);
        Logger.recordOutput("Pivot/atPositionSetpoint", atPositionSetpoint().orElse(false));

        switch (commandedControlMode) {
            case DutyCycle:
                logSetpoints(commandedDutyCycleSetpoint, Volts.of(0.0), Degrees.of(0.0));
                break;
            case Voltage:
                logSetpoints(0.0, commandedVoltageSetpoint, Degrees.of(0.0));
                break;
            case Position:
                logSetpoints(0.0, Volts.of(0.0), commandedPositionSetpoint);
                break;
            default:
                logSetpoints(0.0, Volts.of(0.0), Degrees.of(0.0));
                break;
        }

        pivotMech.setAngle(inputs.encoderAbsolutePosition.in(Degrees));

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> io.updatePID(values[0], values[1], values[2], values[3], values[4], values[5]),
            kP, kI, kD, kS, kV, kA);
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

    public Command manualSpeedCommand(DoubleSupplier speedSupplier) {
        return run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(interrupted -> stop());
    }

    public Command runPivotOut() {
        return this.runOnce(() -> {
            setPosition(IntakeConstants.PIVOT_OUT_POS);
        });
    }

    public Command runPivotIn() {
        return this.runOnce(() -> {
            setPosition(IntakeConstants.PIVOT_IN_POS);
        });
    }

    public Command jigglePivot() {
        Timer jiggleTimer = new Timer();
        AtomicBoolean atLowerSetpoint = new AtomicBoolean(false);
        return this.startRun(
            () -> {
                jiggleTimer.restart();
                setPosition(IntakeConstants.PIVOT_MID_LOWER);
                atLowerSetpoint.set(true);
            },
            () -> {
                if (jiggleTimer.advanceIfElapsed(3)) {
                    if (!atLowerSetpoint.get()) {
                        setPosition(IntakeConstants.PIVOT_MID_LOWER);
                        atLowerSetpoint.set(true);
                    } else {
                        setPosition(IntakeConstants.PIVOT_MID_UPPER);
                        atLowerSetpoint.set(false);
                    }

                }
            });
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            stop();
        });
    }
}
