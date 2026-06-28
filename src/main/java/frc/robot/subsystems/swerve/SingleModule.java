package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    private final LoggedTunableNumber.Watcher drivePIDWatcher;
    private final LoggedTunableNumber.Watcher driveFeedforwardWatcher;
    private final LoggedTunableNumber.Watcher steerPIDWatcher;

    private SimpleMotorFeedforward driveFeedforwardModel;

    private double commandedDriveFeedforwardVolts = 0.0;
    private double commandedDriveVelocityRPS = 0.0;

    private double commandedSteerPositionRot = 0.0;


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

        drivePIDWatcher = LoggedTunableNumber.watch(driveKP, driveKI, driveKD);
        driveFeedforwardWatcher = LoggedTunableNumber.watch(driveKS, driveKV);
        steerPIDWatcher = LoggedTunableNumber.watch(steerKP, steerKI, steerKD, steerKS, steerKV);

        io.setDrivePID(driveKP.get(), driveKI.get(), driveKD.get());
        io.setSteerPID(steerKP.get(), steerKI.get(), steerKD.get(), steerKS.get(), steerKV.get());
    }

    public void setModuleState(SwerveModuleState state) {
        Rotation2d currentAngle = Rotation2d.fromRotations(inputs.encoderAbsolutePositionRot);
        state.optimize(currentAngle);
        state.cosineScale(currentAngle);

        double idealDriveVelocityRPS = state.speedMetersPerSecond / SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS;

        commandedSteerPositionRot = state.angle.getRotations();
        commandedDriveVelocityRPS = idealDriveVelocityRPS;
        commandedDriveFeedforwardVolts = driveFeedforwardModel.calculate(idealDriveVelocityRPS);

        io.setDriveVelocity(commandedDriveVelocityRPS, commandedDriveFeedforwardVolts);
        io.setSteerPosition(commandedSteerPositionRot);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            inputs.driveVelocityRPS * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS,
            getAbsoluteAngle());
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(inputs.encoderAbsolutePositionRot);
    }

    public double getDriveAngVelocityRPS() {
        return inputs.driveVelocityRPS;
    }

    public double getDriveLinVelocityMPS() {
        return inputs.driveVelocityRPS * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE_METERS;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/" + io.getModule().toKey(), inputs);

        drivePIDWatcher.ifChanged(
            () -> io.setDrivePID(driveKP.get(), driveKI.get(), driveKD.get()));

        driveFeedforwardWatcher.ifChanged(
            () -> driveFeedforwardModel = new SimpleMotorFeedforward(driveKS.get(), driveKV.get()));

        steerPIDWatcher.ifChanged(
            () -> io.setSteerPID(steerKP.get(), steerKI.get(), steerKD.get(), steerKS.get(), steerKV.get()));
    }

}
