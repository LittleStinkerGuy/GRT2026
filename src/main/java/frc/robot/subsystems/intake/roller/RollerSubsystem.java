package frc.robot.subsystems.intake.roller;

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
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedSetpointTracker;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;
import frc.robot.util.RollerMechanism2D;
import frc.robot.util.ComponentStatus.MotorControlMode;

public class RollerSubsystem extends SubsystemBase {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber inSpeed =
        new LoggedTunableNumber("Roller/InSpeed_rps", Math.abs(IntakeConstants.ROLLER_IN_SPEED_RPS));
    private final LoggedTunableNumber outSpeed =
        new LoggedTunableNumber("Roller/OutSpeed_rps", Math.abs(IntakeConstants.ROLLER_OUT_SPEED_RPS));

    private final LoggedSetpointTracker setpointTracker = new LoggedSetpointTracker(
        "Roller",
        MotorControlMode.DutyCycle,
        MotorControlMode.Voltage,
        MotorControlMode.Velocity);
    private double commandedVoltageSetpoint = 0.0;
    private double commandedVelocitySetpoint = 0.0;

    private final SysIdRoutine sysIdRoutine;

    private final RollerMechanism2D mechanism = new RollerMechanism2D(0.2);
    @AutoLogOutput(key = "Roller/Mechanism2D")
    private final LoggedMechanism2d mechanism2d = mechanism.getMechanism2d();

    public RollerSubsystem(RollerIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Roller/kP", pid.kP());
        kI = new LoggedTunableNumber("Roller/kI", pid.kI());
        kD = new LoggedTunableNumber("Roller/kD", pid.kD());
        kS = new LoggedTunableNumber("Roller/kS", pid.kS());
        kV = new LoggedTunableNumber("Roller/kV", pid.kV());
        kA = new LoggedTunableNumber("Roller/kA", pid.kA());

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds), // ramp rate: 1 V/s
                Volts.of(7), // step voltage
                Seconds.of(10), // timeout
                (state) -> Logger.recordOutput("Roller/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> setVoltage(voltage),
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
        return Optional.of(Math.abs(commandedVelocitySetpoint - inputs.velocityRPS) <= 5.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Roller", inputs);

        if (DriverStation.isDisabled()) {
            setpointTracker.setControlMode(MotorControlMode.Disabled);
        }

        setpointTracker.logAll();
        Logger.recordOutput("Roller/atVelocitySetpoint", atSetpoint().orElse(false));

        mechanism.setPosition(inputs.positionRot);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> io.updatePID(values[0], values[1], values[2], values[3], values[4], values[5]),
            kP, kI, kD, kS, kV, kA);

        LoggedTracer.record("Roller");
    }

    public Command runSysID() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command setRollerManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command runRollerIn() {
        return this.runEnd(
            () -> setVelocity(Math.abs(inSpeed.get())),
            this::stop);
    }

    public Command runRollerOut() {
        return this.runEnd(
            () -> setVelocity(-Math.abs(outSpeed.get())),
            this::stop);
    }

    public Command stopRoller() {
        return this.runOnce(this::stop);
    }
}
