package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PDHSubsystem extends SubsystemBase {

    private final PowerDistribution pdh;
    private static final String LOG_PREFIX = "PDH/";

    public PDHSubsystem() {
        pdh = new PowerDistribution(1, ModuleType.kRev);

        SmartDashboard.setDefaultBoolean(LOG_PREFIX + "BrownOut", false);
    }

    @Override
    public void periodic() {
        sendData();
    }

    public void sendData() {
        Logger.recordOutput(LOG_PREFIX + "TotalCurrentAmps",
            pdh.getTotalCurrent());

        Logger.recordOutput(LOG_PREFIX + "TotalPowerWatts",
            pdh.getTotalPower());

        Logger.recordOutput(LOG_PREFIX + "TotalEnergyJoules",
            pdh.getTotalEnergy());

        Logger.recordOutput(LOG_PREFIX + "VoltageVolts",
            pdh.getVoltage());

        Logger.recordOutput(LOG_PREFIX + "TemperatureC",
            pdh.getTemperature());

        Logger.recordOutput(LOG_PREFIX + "SwitchableChannelEnabled",
            pdh.getSwitchableChannel());

        SmartDashboard.putBoolean(LOG_PREFIX + "BrownOut", pdh.getStickyFaults().Brownout);

        // Per-channel current logging
        for (int i = 0; i < 24; i++) {
            Logger.recordOutput(LOG_PREFIX + "Channel" + i + "CurrentAmps",
                pdh.getCurrent(i));
        }
    }

    public double getTotalCurrent() {
        return pdh.getTotalCurrent();
    }

    public double getVoltage() {
        return pdh.getVoltage();
    }

    public double getChannelCurrent(int channel) {
        return pdh.getCurrent(channel);
    }

    public void setSwitchableChannel(boolean enabled) {
        pdh.setSwitchableChannel(enabled);
    }
}
