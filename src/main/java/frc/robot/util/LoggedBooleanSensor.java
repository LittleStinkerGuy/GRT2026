package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LoggingConstants;

public class LoggedBooleanSensor {

    private final DigitalInput sensor;
    private final String name;

    private NetworkTableInstance ntInstance;
    private NetworkTable sensorStatsTable;
    private BooleanPublisher sensorReadingPublisher;

    public LoggedBooleanSensor(String name, int port) {
        this.sensor = new DigitalInput(port);
        this.name = name;
        initNt(name);
    }

    /**
     * Get the reading of the sensor
     * 
     * @return
     */
    public boolean get() {
        return sensor.get();
    }

    /**
     * Publish sensor reading to NT
     */
    public void publishStats() {
        sensorReadingPublisher.set(sensor.get());
    }

    /**
     * Logs sensor reading
     */
    public void logStats() {
        Logger.recordOutput(name, sensor.get());
    }

    /**
     * Initialize networktables
     * 
     * @param name
     */
    private void initNt(String name) {
        ntInstance = NetworkTableInstance.getDefault();
        sensorStatsTable = ntInstance.getTable(LoggingConstants.SENSOR_TABLE);
        sensorReadingPublisher = sensorStatsTable.getBooleanTopic(name).publish();
    }
}
