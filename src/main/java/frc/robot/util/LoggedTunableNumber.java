// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage

package frc.robot.util;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import frc.robot.Constants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 * Adapted from team 6328 Mechanical Advantage's 2025 codebase
 */
public class LoggedTunableNumber implements DoubleSupplier {
    private static final String TABLE_KEY = "/Tuning";

    private final String key;
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedNetworkNumber dashboardNumber;

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public LoggedTunableNumber(String dashboardKey) {
        this.key = TABLE_KEY + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (Constants.TUNING_MODE) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return Constants.TUNING_MODE ? dashboardNumber.get() : defaultValue;
        }
    }

    public Watcher watcher() {
        return new Watcher(this);
    }

    public static Watcher watch(LoggedTunableNumber... tunableNumbers) {
        return new Watcher(tunableNumbers);
    }

    public static final class Watcher {
        private final LoggedTunableNumber[] numbers;
        private final double[] last;

        private Watcher(LoggedTunableNumber... numbers) {
            this.numbers = numbers;
            this.last = new double[numbers.length];

            for (int i = 0; i < numbers.length; i++) {
                last[i] = numbers[i].get();
            }
        }

        public boolean changed() {
            boolean changed = false;
            for (int i = 0; i < numbers.length; i++) {
                double current = numbers[i].get();
                if (current != last[i]) {
                    changed = true;
                    last[i] = current;
                }
            }
            return changed;
        }

        public void ifChanged(Consumer<double[]> action) {
            if (!Constants.TUNING_MODE) {
                return;
            }
            if (changed()) {
                action.accept(Arrays.stream(numbers).mapToDouble(LoggedTunableNumber::get).toArray());
            }
        }

        public void ifChanged(Runnable action) {
            if (!Constants.TUNING_MODE) {
                return;
            }
            if (changed()) {
                action.run();
            }
        }
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}
