package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
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
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;
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
        new LoggedTunableNumber("Roller/InSpeed_rps", Math.abs(IntakeConstants.ROLLER_IN_SPEED));
    private final LoggedTunableNumber outSpeed =
        new LoggedTunableNumber("Roller/OutSpeed_rps", Math.abs(IntakeConstants.ROLLER_OUT_SPEED));

    private MotorControlMode commandedControlMode = MotorControlMode.Disabled;
    private AngularVelocity commandedVelocitySetpoint = RotationsPerSecond.of(0.0);

    private final SysIdRoutine sysIdRoutine;

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
        commandedControlMode = MotorControlMode.DutyCycle;
        Logger.recordOutput("Roller/DutyCycleSetpoint", speed);
    }

    public void setVoltage(Voltage volts) {
        volts = Volts.of(MathUtil.clamp(volts.in(Volts), -12.0, 12.0));
        io.setVoltageOut(volts);
        commandedControlMode = MotorControlMode.Voltage;
        Logger.recordOutput("Roller/VoltageSetpoint", volts);
    }

    public void setVelocity(AngularVelocity velo) {
        io.setVelocityOut(velo);
        commandedControlMode = MotorControlMode.Velocity;
        commandedVelocitySetpoint = velo;
        Logger.recordOutput("Roller/VelocitySetpoint", velo);
    }

    public void setVelocity(double rps) {
        setVelocity(RotationsPerSecond.of(rps));
    }

    public void stop() {
        io.stop();
        commandedControlMode = MotorControlMode.Disabled;
    }

    public void runIn() {
        setVelocity(RotationsPerSecond.of(-Math.abs(inSpeed.get())));
    }

    public void runOut() {
        setVelocity(RotationsPerSecond.of(Math.abs(outSpeed.get())));
    }

    public Optional<Boolean> atSetpoint() {
        if (commandedControlMode != MotorControlMode.Velocity) {
            return Optional.empty();
        }
        return Optional.of(commandedVelocitySetpoint.isNear(inputs.velocity, RotationsPerSecond.of(5)));
    }

    public boolean isRunning() {
        return Math.abs(inputs.appliedDutyCycle) > 0.01;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Roller", inputs);

        Logger.recordOutput("Roller/controlMode", commandedControlMode);
        Logger.recordOutput("Roller/atVelocitySetpoint", atSetpoint().orElse(false));

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

    public Command setRollerManualSpeed(DoubleSupplier speedSupplier) {
        return run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(interrupted -> stop());
    }

    public Command runRollerIn() {
        return this.runEnd(this::runIn, this::stop);
    }

    public Command runRollerOut() {
        return this.runEnd(this::runOut, this::stop);
    }

    public Command stopRoller() {
        return this.runOnce(this::stop);
    }
}
