package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.CANType;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;
import org.littletonrobotics.junction.Logger;

// Borrowed update idea in a new thread from team FRC 6328
public class LoggedCanivore extends CANBus {
    private static CopyOnWriteArrayList<LoggedCanivore> canivores = new CopyOnWriteArrayList<>();

    private final CANType canType;
    private final String logPrefix;

    private final Thread thread;
    private volatile Optional<CANBusStatus> canivoreStatus = Optional.empty();

    public LoggedCanivore(CANType canType) {
        super(canType.busName());

        if (canType == CANType.RIO) {
            throw new IllegalArgumentException("Cannot express native RIO Bus as LoggedCanivore");
        }

        this.canType = canType;
        this.logPrefix = "CANBus/" + canType.busName();

        thread =
            new Thread(
                () -> {
                    while (!Thread.currentThread().isInterrupted()) {
                        canivoreStatus = Optional.of(getStatus());
                        try {
                            Thread.sleep(400);
                        } catch (InterruptedException e) {
                            break;
                        }
                    }
                });
        thread.setName(canType.busName() + "Reader");
        thread.setDaemon(true);
        thread.start();

        canivores.add(this);
    }

    public CANType getCanType() {
        return canType;
    }

    public Optional<CANBusStatus> getCANBusStatus() {
        return canivoreStatus;
    }

    private void updateDashboard() {
        Optional<CANBusStatus> status = getCANBusStatus();
        if (status.isPresent()) {
            var currentStatus = status.get();
            Logger.recordOutput(logPrefix + "/Status", currentStatus.Status.getName());
            Logger.recordOutput(logPrefix + "/BusUtilization", currentStatus.BusUtilization);
            Logger.recordOutput(logPrefix + "/BusOffCount", currentStatus.BusOffCount);
            Logger.recordOutput(logPrefix + "/TxFullCount", currentStatus.TxFullCount);
            Logger.recordOutput(logPrefix + "/REC", currentStatus.REC);
            Logger.recordOutput(logPrefix + "/TEC", currentStatus.TEC);
        }
    }

    public static void updateCanivoreStatuses() {
        canivores.forEach((canivore) -> canivore.updateDashboard());
    }
}
