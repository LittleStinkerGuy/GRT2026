// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Drop-in {@link CommandPS5Controller} subclass that routes every PS5-named
 * method to a {@link CommandXboxController} bound to the same port. Lets sim
 * builds use an Xbox controller without changing call sites that already use
 * PS5 button names.
 *
 * <p>
 * Mappings:
 * 
 * <pre>
 *   cross    -> A           L1    -> leftBumper       L3    -> leftStick
 *   circle   -> B           R1    -> rightBumper      R3    -> rightStick
 *   square   -> X           L2    -> leftTrigger>0.5  options -> start
 *   triangle -> Y           R2    -> rightTrigger>0.5 create  -> back
 *   povX     -> povX        getL2Axis -> leftTriggerAxis remapped to [-1, 1]
 *                           getR2Axis -> rightTriggerAxis remapped to [-1, 1]
 * </pre>
 *
 * <p>
 * Touchpad / PS / mute have no Xbox equivalent and return a never-true
 * Trigger.
 */
public class PS5ControllerEmulator extends CommandPS5Controller {

    private static final double TRIGGER_BUTTON_THRESHOLD = 0.5;

    private final CommandXboxController xbox;

    public PS5ControllerEmulator(int port) {
        super(port);
        this.xbox = new CommandXboxController(port);
    }

    /** Underlying Xbox controller, in case raw access is needed. */
    public CommandXboxController getXbox() {
        return xbox;
    }

    // ==================== FACE BUTTONS ====================

    @Override
    public Trigger cross() {
        return xbox.a();
    }

    @Override
    public Trigger circle() {
        return xbox.b();
    }

    @Override
    public Trigger square() {
        return xbox.x();
    }

    @Override
    public Trigger triangle() {
        return xbox.y();
    }

    // ==================== BUMPERS / TRIGGERS ====================

    @Override
    public Trigger L1() {
        return xbox.leftBumper();
    }

    @Override
    public Trigger R1() {
        return xbox.rightBumper();
    }

    /** L2 as a button — fires when the Xbox left trigger crosses 0.5. */
    @Override
    public Trigger L2() {
        return xbox.leftTrigger(TRIGGER_BUTTON_THRESHOLD);
    }

    /** R2 as a button — fires when the Xbox right trigger crosses 0.5. */
    @Override
    public Trigger R2() {
        return xbox.rightTrigger(TRIGGER_BUTTON_THRESHOLD);
    }

    @Override
    public Trigger L3() {
        return xbox.leftStick();
    }

    @Override
    public Trigger R3() {
        return xbox.rightStick();
    }

    // ==================== CENTER / SYSTEM BUTTONS ====================

    @Override
    public Trigger options() {
        return xbox.start();
    }

    @Override
    public Trigger create() {
        return xbox.back();
    }

    /** No Xbox equivalent — never fires. */
    @Override
    public Trigger touchpad() {
        return new Trigger(() -> false);
    }

    // ==================== POV ====================

    @Override
    public Trigger povUp() {
        return xbox.povUp();
    }

    @Override
    public Trigger povUpRight() {
        return xbox.povUpRight();
    }

    @Override
    public Trigger povRight() {
        return xbox.povRight();
    }

    @Override
    public Trigger povDownRight() {
        return xbox.povDownRight();
    }

    @Override
    public Trigger povDown() {
        return xbox.povDown();
    }

    @Override
    public Trigger povDownLeft() {
        return xbox.povDownLeft();
    }

    @Override
    public Trigger povLeft() {
        return xbox.povLeft();
    }

    @Override
    public Trigger povUpLeft() {
        return xbox.povUpLeft();
    }

    @Override
    public Trigger povCenter() {
        return xbox.povCenter();
    }

    // ==================== AXES ====================

    @Override
    public double getLeftX() {
        return xbox.getLeftX();
    }

    @Override
    public double getLeftY() {
        return xbox.getLeftY();
    }

    @Override
    public double getRightX() {
        return xbox.getRightX();
    }

    @Override
    public double getRightY() {
        return xbox.getRightY();
    }

    /**
     * PS5-style L2 axis: -1 at rest, +1 fully pressed. Remapped from the
     * Xbox left trigger's [0, 1] range so existing math like
     * {@code (getR2Axis() + 1) / 3} keeps working.
     */
    @Override
    public double getL2Axis() {
        return xbox.getLeftTriggerAxis() * 2.0 - 1.0;
    }

    /** PS5-style R2 axis: -1 at rest, +1 fully pressed. */
    @Override
    public double getR2Axis() {
        return xbox.getRightTriggerAxis() * 2.0 - 1.0;
    }
}
