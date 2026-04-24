// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import frc.robot.Constants.CANType;
import java.util.EnumMap;
import java.util.List;
import java.util.function.Supplier;

/**
 * Class to refresh all phoenix status signals
 * Taken from 6328's 2025 code
 */
public class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) {
                break;
            }
        }
    }

    /** Signals for synchronized refresh, keyed by bus. */
    private static final EnumMap<CANType, BaseStatusSignal[]> signalsByBus = new EnumMap<>(CANType.class);

    static {
        for (CANType type : CANType.values()) {
            signalsByBus.put(type, new BaseStatusSignal[0]);
        }
    }

    /** Registers a set of signals for synchronized refresh. */
    public static void registerSignals(CANType canType, BaseStatusSignal... signals) {
        BaseStatusSignal[] current = signalsByBus.get(canType);
        BaseStatusSignal[] combined = new BaseStatusSignal[current.length + signals.length];
        System.arraycopy(current, 0, combined, 0, current.length);
        System.arraycopy(signals, 0, combined, current.length, signals.length);
        signalsByBus.put(canType, combined);
    }

    public static void registerSignals(CANType canType, List<BaseStatusSignal> signals) {
        registerSignals(canType, signals.toArray(new BaseStatusSignal[0]));
    }

    /** Refresh all registered signals. */
    public static void refreshAllStatusSignals() {
        for (BaseStatusSignal[] signals : signalsByBus.values()) {
            if (signals.length > 0) {
                BaseStatusSignal.refreshAll(signals);
            }
        }
    }
}
