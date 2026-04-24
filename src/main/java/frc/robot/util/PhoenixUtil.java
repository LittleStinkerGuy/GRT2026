// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.CANType;
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

    /** Signals for synchronized refresh. */
    private static BaseStatusSignal[] swerveCANSignals = new BaseStatusSignal[0];
    private static BaseStatusSignal[] mechCANSignals = new BaseStatusSignal[0];
    private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

    /** Registers a set of signals for synchronized refresh. */
    public static void registerSignals(CANType canType, BaseStatusSignal... signals) {
        switch (canType) {
            case RIO -> {
                BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
                System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
                System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
                rioSignals = newSignals;
            }
            case SWERVE -> {
                BaseStatusSignal[] newSignals = new BaseStatusSignal[swerveCANSignals.length + signals.length];
                System.arraycopy(swerveCANSignals, 0, newSignals, 0, swerveCANSignals.length);
                System.arraycopy(signals, 0, newSignals, swerveCANSignals.length, signals.length);
                swerveCANSignals = newSignals;
            }
            case MECH -> {
                BaseStatusSignal[] newSignals = new BaseStatusSignal[mechCANSignals.length + signals.length];
                System.arraycopy(mechCANSignals, 0, newSignals, 0, mechCANSignals.length);
                System.arraycopy(signals, 0, newSignals, mechCANSignals.length, signals.length);
                mechCANSignals = newSignals;
            }
            default -> {
                try (Alert missingTypeAlert = new Alert("Missing Canivore Type in PhoenixUtil!!", AlertType.kError)) {
                    missingTypeAlert.set(true);
                }
                System.out.println("Missing Canivore Type in PhoenixUtil!!");
            }
        }
    }

    public static void registerSignals(CANType canType, List<BaseStatusSignal> signals) {
        registerSignals(canType, signals.toArray(new BaseStatusSignal[0]));
    }

    /** Refresh all registered signals. */
    public static void refreshAllStatusSignals() {
        if (swerveCANSignals.length > 0) {
            BaseStatusSignal.refreshAll(swerveCANSignals);
        }
        if (mechCANSignals.length > 0) {
            BaseStatusSignal.refreshAll(mechCANSignals);
        }
        if (rioSignals.length > 0) {
            BaseStatusSignal.refreshAll(rioSignals);
        }
    }
}
