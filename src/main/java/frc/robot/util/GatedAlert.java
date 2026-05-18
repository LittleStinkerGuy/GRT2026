package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import java.util.function.BooleanSupplier;

/**
 * Alert that fires only while a gate is open. Use to suppress cascading noise — e.g. silence
 * a motor's config-failure alerts while the motor itself is disconnected.
 *
 * <p>
 * Drop-in for {@link Alert}: pass into anything that takes an Alert, including
 * {@link PhoenixUtil#tryUntilOk(int, java.util.function.Supplier, Alert)}. When the gate
 * closes, currently-raised alerts are silenced; the underlying raised state is preserved so
 * reopening the gate restores any still-valid alerts.
 *
 * <p>
 * The owner must call {@link #push()} when the gate state changes, since the gate is
 * polled, not observed. In practice this means the gate should only be flipped by the same
 * code that calls {@link #push()} immediately afterward — otherwise alerts can lag by a
 * frame (or stay stuck across a missed flip).
 */
public class GatedAlert extends Alert {
    private final BooleanSupplier gate;
    private boolean actualValue = false;

    public GatedAlert(String message, AlertType type, BooleanSupplier gate) {
        super(message, type);
        this.gate = gate;
    }

    @Override
    public void set(boolean actualValue) {
        this.actualValue = actualValue;
        push();
    }

    /** Re-evaluate against the current gate state without changing the underlying raised state. */
    public void push() {
        super.set(actualValue && gate.getAsBoolean());
    }
}
