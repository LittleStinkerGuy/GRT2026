package frc.robot.subsystems.shooter.tower;

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
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.TowerConstants.TowerIntake;
import frc.robot.util.ComponentStatus.MotorControlMode;
import frc.robot.util.LoggedTunableNumber;
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
    private AngularVelocity commandedVelocitySetpoint = RotationsPerSecond.of(0.0);

    private final SysIdRoutine sysIdRoutine;

    private final RollerMechanism2D mechanism = new RollerMechanism2D(0.3);

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
        commandedControlMode = MotorControlMode.DutyCycle;
        Logger.recordOutput("Tower/DutyCycleSetpoint", speed);
    }

    public void setVoltage(Voltage volts) {
        volts = Volts.of(MathUtil.clamp(volts.in(Volts), -12.0, 12.0));
        io.setVoltageOut(volts);
        commandedControlMode = MotorControlMode.Voltage;
        Logger.recordOutput("Tower/VoltageSetpoint", volts);
    }

    public void setVelocity(AngularVelocity velo) {
        io.setVelocityOut(velo);
        commandedControlMode = MotorControlMode.Velocity;
        commandedVelocitySetpoint = velo;
        Logger.recordOutput("Tower/VelocitySetpoint", velo);
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

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Tower", inputs);

        Logger.recordOutput("Tower/controlMode", commandedControlMode);
        Logger.recordOutput("Tower/atVelocitySetpoint", atSetpoint().orElse(false));

        mechanism.setPosition(inputs.position);
        Logger.recordOutput("Tower/Mechanism2D", mechanism.getMechanism2d());

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
