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
 * Mappings (PS5 → Xbox):
 *
 * <pre>
 *   ┌──────────────┬─────────────────────────────┐
 *   │  PS5         │  Xbox                       │
 *   ├──────────────┼─────────────────────────────┤
 *   │  cross       │  A                          │
 *   │  circle      │  B                          │
 *   │  square      │  X                          │
 *   │  triangle    │  Y                          │
 *   │  L1   / R1   │  Left / Right Bumper        │
 *   │  L2   / R2   │  Left / Right Trigger       │
 *   │  L3   / R3   │  Left / Right Stick Click   │
 *   │  options     │  Menu  (a.k.a. Start)       │
 *   │  create      │  View  (a.k.a. Back)        │
 *   │  touchpad    │  Share    (macOS only)      │
 *   │  povX        │  POV X / D-Pad              │
 *   │  leftX  / Y  │  Left  Stick X / Y          │
 *   │  rightX / Y  │  Right Stick X / Y          │
 *   └──────────────┴─────────────────────────────┘
 * </pre>
 *
 * <p>
 * Trigger axes ({@link #getL2Axis()} / {@link #getR2Axis()}) read
 * {@code [0, 1]} (rest = 0, fully pressed = 1). The {@link #L2()} /
 * {@link #R2()} button triggers fire past 50% press. The PS / mute buttons
 * have no Xbox equivalent and return a never-true Trigger.
 *
 * <p>
 * On macOS the native Xbox HID exposes a different axis order and button
 * layout than the XInput layout WPILib's {@code CommandXboxController}
 * assumes. This class detects the platform and reads raw axis / button
 * indices when needed — see the {@code MAC_*} constants below.
 */
public class PS5ControllerEmulator extends CommandPS5Controller {

    private static final double TRIGGER_BUTTON_THRESHOLD = 0.5;

    // macOS's native Xbox HID layout (all sticks & triggers are -1..+1):
    // axes: 0=LX 1=LY 2=RX 3=RY 4=RT 5=LT
    // buttons (1-indexed):
    // 1=A 2=B 3=- 4=X 5=Y 6=- 7=LB 8=RB 9=- 10=-
    // 11=View 12=Options 13=LStickClick 14=RStickClick 15=Share
    // WPILib's CommandXboxController assumes the XInput layout, which disagrees on
    // every axis past LY and most buttons. On macOS we bypass it and read raw
    // axes/buttons directly.
    private static final boolean IS_MACOS =
        System.getProperty("os.name", "").toLowerCase().startsWith("mac");

    private static final int MAC_AXIS_LX = 0;
    private static final int MAC_AXIS_LY = 1;
    private static final int MAC_AXIS_RX = 2;
    private static final int MAC_AXIS_RY = 3;
    private static final int MAC_AXIS_RT = 4;
    private static final int MAC_AXIS_LT = 5;

    private static final int MAC_BTN_A = 1;
    private static final int MAC_BTN_B = 2;
    private static final int MAC_BTN_X = 4;
    private static final int MAC_BTN_Y = 5;
    private static final int MAC_BTN_LB = 7;
    private static final int MAC_BTN_RB = 8;
    private static final int MAC_BTN_VIEW = 11; // PS5 "create"
    private static final int MAC_BTN_OPTIONS = 12;
    private static final int MAC_BTN_LSTICK = 14;
    private static final int MAC_BTN_RSTICK = 15;
    private static final int MAC_BTN_SHARE = 16;

    private final CommandXboxController xbox;

    public PS5ControllerEmulator(int port) {
        super(port);
        this.xbox = new CommandXboxController(port);
    }

    private Trigger macButton(int idx) {
        return new Trigger(() -> xbox.getHID().getRawButton(idx));
    }

    /** Underlying Xbox controller, in case raw access is needed. */
    public CommandXboxController getXbox() {
        return xbox;
    }

    // ==================== FACE BUTTONS ====================

    @Override
    public Trigger cross() {
        return IS_MACOS ? macButton(MAC_BTN_A) : xbox.a();
    }

    @Override
    public Trigger circle() {
        return IS_MACOS ? macButton(MAC_BTN_B) : xbox.b();
    }

    @Override
    public Trigger square() {
        return IS_MACOS ? macButton(MAC_BTN_X) : xbox.x();
    }

    @Override
    public Trigger triangle() {
        return IS_MACOS ? macButton(MAC_BTN_Y) : xbox.y();
    }

    // ==================== BUMPERS / TRIGGERS ====================

    @Override
    public Trigger L1() {
        return IS_MACOS ? macButton(MAC_BTN_LB) : xbox.leftBumper();
    }

    @Override
    public Trigger R1() {
        return IS_MACOS ? macButton(MAC_BTN_RB) : xbox.rightBumper();
    }

    /** L2 as a button — fires when the left trigger crosses 50% pressed. */
    @Override
    public Trigger L2() {
        return new Trigger(() -> getL2Axis() > TRIGGER_BUTTON_THRESHOLD);
    }

    /** R2 as a button — fires when the right trigger crosses 50% pressed. */
    @Override
    public Trigger R2() {
        return new Trigger(() -> getR2Axis() > TRIGGER_BUTTON_THRESHOLD);
    }

    @Override
    public Trigger L3() {
        return IS_MACOS ? macButton(MAC_BTN_LSTICK) : xbox.leftStick();
    }

    @Override
    public Trigger R3() {
        return IS_MACOS ? macButton(MAC_BTN_RSTICK) : xbox.rightStick();
    }

    // ==================== CENTER / SYSTEM BUTTONS ====================

    @Override
    public Trigger options() {
        return IS_MACOS ? macButton(MAC_BTN_OPTIONS) : xbox.start();
    }

    @Override
    public Trigger create() {
        return IS_MACOS ? macButton(MAC_BTN_VIEW) : xbox.back();
    }

    /** On macOS, mapped to the Xbox Share button. Elsewhere, never fires. */
    @Override
    public Trigger touchpad() {
        return IS_MACOS ? macButton(MAC_BTN_SHARE) : new Trigger(() -> false);
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
        if (IS_MACOS) {
            return xbox.getHID().getRawAxis(MAC_AXIS_LX);
        }
        return xbox.getLeftX();
    }

    @Override
    public double getLeftY() {
        if (IS_MACOS) {
            return xbox.getHID().getRawAxis(MAC_AXIS_LY);
        }
        return xbox.getLeftY();
    }

    @Override
    public double getRightX() {
        if (IS_MACOS) {
            return xbox.getHID().getRawAxis(MAC_AXIS_RX);
        }
        return xbox.getRightX();
    }

    @Override
    public double getRightY() {
        if (IS_MACOS) {
            return xbox.getHID().getRawAxis(MAC_AXIS_RY);
        }
        return xbox.getRightY();
    }

    /** PS5 L2 axis: 0 at rest, 1 fully pressed (matches CommandPS5Controller). */
    @Override
    public double getL2Axis() {
        if (IS_MACOS) {
            // macOS reports the trigger as -1 (rest) → +1 (pressed); remap to [0, 1].
            return (xbox.getHID().getRawAxis(MAC_AXIS_LT) + 1.0) / 2.0;
        }
        return xbox.getLeftTriggerAxis();
    }

    /** PS5 R2 axis: 0 at rest, 1 fully pressed (matches CommandPS5Controller). */
    @Override
    public double getR2Axis() {
        if (IS_MACOS) {
            return (xbox.getHID().getRawAxis(MAC_AXIS_RT) + 1.0) / 2.0;
        }
        return xbox.getRightTriggerAxis();
    }
}
