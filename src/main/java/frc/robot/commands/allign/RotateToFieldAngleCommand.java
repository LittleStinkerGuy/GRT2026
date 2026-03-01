package frc.robot.commands.allign;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateToAngleConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RotateToFieldAngleCommand extends Command {
    private final SwerveSubsystem swerve;
    private final PIDController pid;
    private final double targetDegrees;
    private final BooleanSupplier cancelCondition;

    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("AutoAim");
    private static final NetworkTableEntry kPEntry = table.getEntry("kP");
    private static final NetworkTableEntry kIEntry = table.getEntry("kI");
    private static final NetworkTableEntry kDEntry = table.getEntry("kD");
    private static final NetworkTableEntry toleranceEntry = table.getEntry("Tolerance");
    private static final NetworkTableEntry goalEntry = table.getEntry("Goal");
    private static final NetworkTableEntry actualEntry = table.getEntry("Actual");
    private static final NetworkTableEntry errorEntry = table.getEntry("Error");
    private static final NetworkTableEntry outputEntry = table.getEntry("Output");

    private static boolean initialized = false;

    public RotateToFieldAngleCommand(SwerveSubsystem swerve, double targetDegrees, BooleanSupplier cancelCondition) {
        this.swerve = swerve;
        this.targetDegrees = targetDegrees;

        this.cancelCondition = cancelCondition;
        this.pid = new PIDController(
            RotateToAngleConstants.kP,
            RotateToAngleConstants.kI,
            RotateToAngleConstants.kD
        );
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(RotateToAngleConstants.TOLERANCE_DEGREES);
        addRequirements(swerve);

        if (!initialized) {
            kPEntry.setDouble(RotateToAngleConstants.kP);
            kIEntry.setDouble(RotateToAngleConstants.kI);
            kDEntry.setDouble(RotateToAngleConstants.kD);
            toleranceEntry.setDouble(RotateToAngleConstants.TOLERANCE_DEGREES);

            goalEntry.setDouble(Double.NaN);
            actualEntry.setDouble(Double.NaN);
            errorEntry.setDouble(Double.NaN);
            outputEntry.setDouble(Double.NaN);
            initialized = true;
        }
    }

    @Override
    public void initialize() {
        pid.setP(kPEntry.getDouble(RotateToAngleConstants.kP));
        pid.setI(kIEntry.getDouble(RotateToAngleConstants.kI));
        pid.setD(kDEntry.getDouble(RotateToAngleConstants.kD));
        pid.setTolerance(toleranceEntry.getDouble(RotateToAngleConstants.TOLERANCE_DEGREES));
        pid.reset();
    }

    @Override
    public void execute() {
        // Live update PID values from NetworkTables
        pid.setP(kPEntry.getDouble(RotateToAngleConstants.kP));
        pid.setI(kIEntry.getDouble(RotateToAngleConstants.kI));
        pid.setD(kDEntry.getDouble(RotateToAngleConstants.kD));
        pid.setTolerance(toleranceEntry.getDouble(RotateToAngleConstants.TOLERANCE_DEGREES));

        double currentAngle = normalizeAngle(swerve.getRobotPosition().getRotation().getDegrees());
        double normalizedTarget = normalizeAngle(targetDegrees);
        double rotationPower = pid.calculate(currentAngle, normalizedTarget);
        swerve.setDrivePowers(0, 0, rotationPower);

        // Publish feedback to NetworkTables
        goalEntry.setDouble(normalizedTarget);
        actualEntry.setDouble(currentAngle);
        double error = normalizedTarget - currentAngle;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        errorEntry.setDouble(error);
        outputEntry.setDouble(rotationPower);
    }

    /**
     * Normalizes an angle to the range [-180, 180]
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || cancelCondition.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setDrivePowers(0, 0, 0);
    }
    

}
