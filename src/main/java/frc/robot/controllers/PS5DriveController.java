package frc.robot.controllers;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.SwerveConstants.*;

public class PS5DriveController extends BaseDriveController {

    private final CommandPS5Controller driveController = new CommandPS5Controller(0);
    private Trigger L1 = new Trigger(driveController.L1());
    private Trigger R1 = new Trigger(driveController.R1());
    private Trigger cross = new Trigger(driveController.cross());
    private Trigger square = new Trigger(driveController.square());
    private double deadZone = 0;

    // Slew rate limiters for smooth acceleration (high negative rate = instant deceleration)
    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT, -1000, 0);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT, -1000, 0);
    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(STEER_SLEW_RATE_LIMIT, -1000, 0);

    @Override
    public double getForwardPower() {
        double forwardPower = -driveController.getLeftY();
        double output = Math.abs(forwardPower) > deadZone ? forwardPower : 0;
        return applyAsymmetricLimit(forwardLimiter, output);
    }

    @Override
    public double getLeftPower() {
        double leftPower = -driveController.getLeftX();
        double output = Math.abs(leftPower) > deadZone ? leftPower : 0;
        return applyAsymmetricLimit(strafeLimiter, output);
    }

    @Override
    public double getRotatePower() {
        double rotatePower = -driveController.getRightX();
        double output = Math.abs(rotatePower) > deadZone ? rotatePower : 0;
        return applyAsymmetricLimit(rotateLimiter, output);
    }

    /**
     * Applies slew rate limiting only for acceleration (increasing magnitude).
     * Deceleration (decreasing magnitude) is instant.
     */
    private double applyAsymmetricLimit(SlewRateLimiter limiter, double input) {
        double limited = limiter.calculate(input);
        // If input magnitude is less than limited magnitude, use input directly (instant decel)
        if (Math.abs(input) < Math.abs(limited)) {
            limiter.reset(input);
            return input;
        }
        return limited;
    }

    @Override
    public boolean getDriverHeadingResetButton() {
        return cross.getAsBoolean();
    }

    @Override
    public boolean getLeftBumper() {
        return L1.getAsBoolean();
    }

    @Override
    public boolean getRightBumper() {
        return R1.getAsBoolean();
    }

    @Override
    public Trigger getRelativeMode() {
        return square;
    }

    public boolean getRightTrigger() {
        return driveController.getR2Axis() > .1;
    }

    public boolean getLeftTrigger() {
        return driveController.getL2Axis() > .1;
    }

    @Override
    public void bindDriverHeadingReset(
            Runnable command, Subsystem requiredSubsystem) {
        // EventLoop eventLoop = new EventLoop();
        InstantCommand instantCommand = new InstantCommand(
                command,
                requiredSubsystem);
        // eventLoop.bind(command);
        // driveController.L1(eventLoop);
        new Trigger(this::getDriverHeadingResetButton).onTrue(instantCommand);
    }

    @Override
    public void setDeadZone(double deadZone) {
        this.deadZone = deadZone;
    }

    public int getPOV() {
        return driveController.getHID().getPOV();
    }

    public Trigger triangle() {
        return driveController.triangle();
    }

    public Trigger circle() {
        return driveController.circle();
    }

    @Override
    public Trigger getAlignToReef() {
        return cross;
    }

    @Override
    public Trigger getAlignToSource() {
        return square;
    }
}
