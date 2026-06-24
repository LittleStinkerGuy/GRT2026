package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants.CANType;
import java.util.concurrent.CopyOnWriteArrayList;
import org.littletonrobotics.junction.Logger;

// Borrowed update idea in a new thread from team FRC 6328
public class LoggedCanivore extends CANBus {
    private static CopyOnWriteArrayList<LoggedCanivore> canivores = new CopyOnWriteArrayList<>();

    private final CANType canType;
    private final String logPrefix;

    private final Thread thread;
    private volatile CANBusStatus canivoreStatus;

    public LoggedCanivore(CANType canType) {
        super(canType.busName());

        if (canType == CANType.RIO) {
            throw new IllegalArgumentException("Cannot express native RIO Bus as LoggedCanivore");
        }

        this.canType = canType;
        this.logPrefix = "LoggedCanivore/" + canType.busName();
        this.canivoreStatus = getStatus();

        thread =
            new Thread(
                () -> {
                    while (true) {
                        canivoreStatus = getStatus();
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
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

    public CANBusStatus getBusStatus() {
        return canivoreStatus;
    }

    private void updateDashboard() {
        var currentStatus = getBusStatus();
        Logger.recordOutput(logPrefix + "/Status", currentStatus.Status.getName());
        Logger.recordOutput(logPrefix + "/BusUtilization", currentStatus.BusUtilization);
        Logger.recordOutput(logPrefix + "/BusOffCount", currentStatus.BusOffCount);
        Logger.recordOutput(logPrefix + "/TxFullCount", currentStatus.TxFullCount);
        Logger.recordOutput(logPrefix + "/REC", currentStatus.REC);
        Logger.recordOutput(logPrefix + "/TEC", currentStatus.TEC);
    }

    public static void updateCanivoreStatuses() {
        canivores.forEach((canivore) -> canivore.updateDashboard());
    }
}
