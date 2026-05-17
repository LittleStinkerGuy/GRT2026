package frc.robot.subsystems.shooter.tower;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.TowerConstants.TowerIntake;
import frc.robot.util.ComponentStatus.MotorControlMode;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MeasureConstants;
import frc.robot.util.PIDConstants;
import frc.robot.util.RollerMechanism2D;

public class TowerSubsystem extends SubsystemBase {
    private final TowerIO io;
    private final TowerIOInputsAutoLogged inputs = new TowerIOInputsAutoLogged();

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber motionMagicAccel =
        new LoggedTunableNumber("Tower/motionMagicAccel_rotPerSec2",
            TowerConstants.MM_ACCEL.in(RotationsPerSecondPerSecond));
    private final LoggedTunableNumber motionMagicVelo =
        new LoggedTunableNumber("Tower/motionMagicVelocity_rotPerSec",
            TowerConstants.MM_MAX_VELO.in(RotationsPerSecond));
    private final LoggedTunableNumber motionMagicJerk =
        new LoggedTunableNumber("Tower/motionMagicJerk_rotPerSec3",
            TowerConstants.MM_JERK.in(RotationsPerSecondPerSecond.per(Second)));

    private MotorControlMode commandedControlMode = MotorControlMode.Disabled;
    private double commandedDutyCycleSetpoint = 0.0;
    private final MutVoltage commandedVoltageSetpoint = Volts.mutable(0.0);
    private final MutAngularVelocity commandedVelocitySetpoint = RotationsPerSecond.mutable(0.0);

    private final SysIdRoutine sysIdRoutine;

    private final RollerMechanism2D mechanism = new RollerMechanism2D(0.3);
    @AutoLogOutput(key = "Tower/Mechanism2D")
    private final LoggedMechanism2d mechanism2d = mechanism.getMechanism2d();

    public TowerSubsystem(TowerIO io) {
        this.io = io;

        PIDConstants pid = io.getDefaultPID();
        kP = new LoggedTunableNumber("Tower/kP", pid.kP());
        kI = new LoggedTunableNumber("Tower/kI", pid.kI());
        kD = new LoggedTunableNumber("Tower/kD", pid.kD());
        kS = new LoggedTunableNumber("Tower/kS", pid.kS());
        kV = new LoggedTunableNumber("Tower/kV", pid.kV());
        kA = new LoggedTunableNumber("Tower/kA", pid.kA());

        io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds), // ramp rate: 1 V/s
                Volts.of(7), // step voltage
                Seconds.of(10), // timeout
                (state) -> Logger.recordOutput("Tower/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this));
    }

    public void setDutyCycle(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        io.setDutyCycleOut(speed);
        commandedDutyCycleSetpoint = speed;
        commandedControlMode = MotorControlMode.DutyCycle;
    }

    public void setVoltage(Voltage volts) {
        commandedVoltageSetpoint.mut_replace(
            MathUtil.clamp(volts.in(Volts), -12.0, 12.0),
            Volts);
        io.setVoltageOut(commandedVoltageSetpoint);
        commandedControlMode = MotorControlMode.Voltage;
    }

    public void setVelocity(AngularVelocity velo) {
        io.setVelocityOut(velo);
        commandedVelocitySetpoint.mut_replace(velo);
        commandedControlMode = MotorControlMode.Velocity;
    }

    public void stop() {
        io.stop();
        commandedControlMode = MotorControlMode.Disabled;
    }

    public Optional<Boolean> atSetpoint() {
        if (commandedControlMode != MotorControlMode.Velocity) {
            return Optional.empty();
        }
        return Optional.of(commandedVelocitySetpoint.isNear(inputs.velocity, TowerConstants.VELOCITY_TOLERANCE));
    }

    public void setTower(TowerIntake state) {
        switch (state) {
            case BALL_UP:
                setVelocity(TowerConstants.TARGET_VELO);
                break;
            case BALL_DOWN:
                setVelocity(TowerConstants.TARGET_VELO.unaryMinus());
                break;
            case STOP:
                stop();
                break;
            default:
                break;
        }
    }

    private void updateSetpoints(double dutyCycle, Voltage voltage, AngularVelocity velocity) {
        commandedDutyCycleSetpoint = dutyCycle;
        commandedVoltageSetpoint.mut_replace(voltage);
        commandedVelocitySetpoint.mut_replace(velocity);
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Tower", inputs);

        if (DriverStation.isDisabled()) {
            commandedControlMode = MotorControlMode.Disabled;
        }

        switch (commandedControlMode) {
            case DutyCycle -> updateSetpoints(commandedDutyCycleSetpoint, MeasureConstants.ZERO_VOLTAGE, MeasureConstants.ZERO_ANGULAR_VELOCITY);
            case Voltage -> updateSetpoints(0.0, commandedVoltageSetpoint, MeasureConstants.ZERO_ANGULAR_VELOCITY);
            case Velocity -> updateSetpoints(0.0, MeasureConstants.ZERO_VOLTAGE, commandedVelocitySetpoint);
            default -> updateSetpoints(0.0, MeasureConstants.ZERO_VOLTAGE, MeasureConstants.ZERO_ANGULAR_VELOCITY);
        }

        Logger.recordOutput("Tower/controlMode", commandedControlMode);
        Logger.recordOutput("Tower/DutyCycleSetpoint", commandedDutyCycleSetpoint);
        Logger.recordOutput("Tower/VoltageSetpoint", commandedVoltageSetpoint);
        Logger.recordOutput("Tower/VelocitySetpoint", commandedVelocitySetpoint);

        Logger.recordOutput("Tower/atVelocitySetpoint", atSetpoint().orElse(false));

        mechanism.setPosition(inputs.position);

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

        LoggedTracer.record("Tower");
    }

    public Command runSysID() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command setTowerManualSpeed(DoubleSupplier speedSupplier) {
        return this.run(() -> setDutyCycle(speedSupplier.getAsDouble())).finallyDo(this::stop);
    }

    public Command runTowerOutput() {
        return this.runEnd(
            () -> setTower(TowerIntake.BALL_DOWN),
            this::stop);
    }

    public Command runTowerInput() {
        return this.runEnd(
            () -> setTower(TowerIntake.BALL_UP),
            this::stop);
    }

    public Command stopTower() {
        return this.runOnce(this::stop);
    }
}
