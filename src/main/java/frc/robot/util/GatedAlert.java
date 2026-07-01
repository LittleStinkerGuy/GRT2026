package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import java.util.concurrent.CopyOnWriteArrayList;
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
 * Every instance self-registers, and {@link #pushAll()} re-evaluates every gate. Call it once
 * per loop (see {@code Robot.robotPeriodic}) so gates are polled automatically — a gate flip
 * then takes effect within one loop and a missed flip can never get stuck. {@link #push()}
 * remains available for callers that want a same-loop update after flipping a gate, but it is
 * no longer required for correctness.
 */
public class GatedAlert extends Alert {
    private static CopyOnWriteArrayList<GatedAlert> registry = new CopyOnWriteArrayList<>();

    private final BooleanSupplier gate;
    private boolean actualValue = false;

    public GatedAlert(String message, AlertType type, BooleanSupplier gate) {
        super(message, type);
        this.gate = gate;
        registry.add(this);
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

    /** Re-evaluate every {@link GatedAlert}'s gate. Call once per loop. */
    public static void pushAll() {
        registry.forEach(GatedAlert::push);
    }
}
