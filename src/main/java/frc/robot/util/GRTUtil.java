package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;

public class GRTUtil {
    /**
     * Get the current time in seconds
     * 
     * @return seconds
     */
    public static long getFpgaTime() {
        return (long) (RobotController.getFPGATime());
    }
}

