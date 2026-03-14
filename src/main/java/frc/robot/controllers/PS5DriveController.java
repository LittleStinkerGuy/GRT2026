package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5DriveController extends BaseDriveController {

    private final CommandPS5Controller driveController = new CommandPS5Controller(0);
    private Trigger L1 = new Trigger(driveController.L1());
    private Trigger R1 = new Trigger(driveController.R1());
    private Trigger cross = new Trigger(driveController.cross());
    private Trigger square = new Trigger(driveController.square());
    private double deadZone = 0;

    @Override
    public double getForwardPower() {
        double forwardPower = -driveController.getLeftY();
        if (Math.abs(forwardPower) > deadZone)
            return -driveController.getLeftY();
        else
            return 0;
    }

    @Override
    public double getLeftPower() {
        double leftPower = -driveController.getLeftX();
        if (Math.abs(leftPower) > deadZone)
            return -driveController.getLeftX();
        else
            return 0;
    }

    @Override
    public double getRotatePower() {
        double rotatePower = -driveController.getRightX();
        if (Math.abs(rotatePower) > deadZone)
            return -driveController.getRightX();
        else
            return 0;
    }

    @Override
    public boolean getDriverHeadingResetButton() {
        return cross.getAsBoolean();
    }

    @Override
    public boolean getLeftBumper() {
        return L1.getAsBoolean();
    }

    public Trigger L1() {
        return L1;
    }

    public Trigger R1() {
        return R1;
    }

    @Override
    public boolean getRightBumper() {
        return R1.getAsBoolean();
    }

    public boolean getRightTrigger() {
        return driveController.getR2Axis() > .1;
    }

    public boolean getLeftTrigger() {
        return driveController.getL2Axis() > .1;
    }

    /**
     * Gets the raw left trigger axis value.
     * 
     * @return Value from 0.0 (not pressed) to 1.0 (fully pressed)
     */
    public double getLeftTriggerAxis() {
        // L2Axis returns -1 to 1, convert to 0 to 1
        return (driveController.getL2Axis() + 1) / 2;
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

    public CommandPS5Controller getController() {
        return driveController;
    }

    public Trigger options() {
        return driveController.options();
    }

    public Trigger square() {
        return square;
    }

    public Trigger create() {
        return driveController.create();
    }
}
