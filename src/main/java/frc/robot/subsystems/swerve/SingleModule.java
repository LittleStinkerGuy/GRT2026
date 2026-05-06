package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PIDConstants;

public class SingleModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    private final LoggedTunableNumber driveKP;
    private final LoggedTunableNumber driveKI;
    private final LoggedTunableNumber driveKD;
    private final LoggedTunableNumber driveKS;
    private final LoggedTunableNumber driveKV;

    private final LoggedTunableNumber steerKP;
    private final LoggedTunableNumber steerKI;
    private final LoggedTunableNumber steerKD;
    private final LoggedTunableNumber steerKS;
    private final LoggedTunableNumber steerKV;

    private SimpleMotorFeedforward driveFeedforwardModel;

    private MutVoltage commandedDriveFeedforward = Volts.mutable(0.0);
    private MutAngularVelocity commandedDriveVelocity = RotationsPerSecond.mutable(0.0);

    private MutAngle commandedSteerPosition = Rotations.mutable(0.0);


    public SingleModule(ModuleIO io) {
        this.io = io;
        PIDConstants drivePID = io.getDefaultDrivePID();
        PIDConstants steerPID = io.getDefaultSteerPID();

        driveFeedforwardModel = new SimpleMotorFeedforward(
            drivePID.kS(), drivePID.kV());

        String tableKey = "Swerve/" + io.getModule().toKey();
        driveKP = new LoggedTunableNumber(tableKey + "/Drive/kP", drivePID.kP());
        driveKI = new LoggedTunableNumber(tableKey + "/Drive/kI", drivePID.kI());
        driveKD = new LoggedTunableNumber(tableKey + "/Drive/kD", drivePID.kD());
        driveKS = new LoggedTunableNumber(tableKey + "/Drive/kS", drivePID.kS());
        driveKV = new LoggedTunableNumber(tableKey + "/Drive/kV", drivePID.kV());

        steerKP = new LoggedTunableNumber(tableKey + "/Steer/kP", steerPID.kP());
        steerKI = new LoggedTunableNumber(tableKey + "/Steer/kI", steerPID.kI());
        steerKD = new LoggedTunableNumber(tableKey + "/Steer/kD", steerPID.kD());
        steerKS = new LoggedTunableNumber(tableKey + "/Steer/kS", steerPID.kS());
        steerKV = new LoggedTunableNumber(tableKey + "/Steer/kV", steerPID.kV());

        io.setDrivePID(driveKP.get(), driveKI.get(), driveKD.get());
        io.setSteerPID(steerKP.get(), steerKI.get(), steerKD.get(), steerKS.get(), steerKV.get());
    }

    public void setModuleState(SwerveModuleState state) {
        Rotation2d currentAngle = Rotation2d.fromRadians(inputs.encoderAbsolutePosition.in(Radians));
        state.optimize(currentAngle);
        state.cosineScale(currentAngle);

        double idealDriveVelocityRadPerSec = state.speedMetersPerSecond / SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS;

        commandedSteerPosition.mut_replace(state.angle.getRadians(), Radians);
        commandedDriveVelocity.mut_replace(idealDriveVelocityRadPerSec, RadiansPerSecond);
        commandedDriveFeedforward.mut_replace(
            driveFeedforwardModel.calculate(idealDriveVelocityRadPerSec / (2 * Math.PI)), Volts);

        io.setDriveVelocity(commandedDriveVelocity, commandedDriveFeedforward);
        io.setSteerPosition(commandedSteerPosition);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            inputs.driveVelocity.in(RadiansPerSecond) * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS,
            getAbsoluteAngle());
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRadians(inputs.encoderAbsolutePosition.in(Radians));
    }

    public AngularVelocity getDriveAngVelocity() {
        return inputs.driveVelocity;
    }

    public LinearVelocity getDriveLinVelocity() {
        return MetersPerSecond.of(
            inputs.driveVelocity.in(RadiansPerSecond) * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/" + io.getModule().toKey(), inputs);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> io.setDrivePID(values[0], values[1], values[2]),
            driveKP, driveKI, driveKD);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> driveFeedforwardModel = new SimpleMotorFeedforward(values[0], values[1]),
            driveKS, driveKV);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            values -> io.setSteerPID(values[0], values[1], values[2], values[3], values[4]),
            steerKP, steerKI, steerKD, steerKS, steerKV);
    }

}
