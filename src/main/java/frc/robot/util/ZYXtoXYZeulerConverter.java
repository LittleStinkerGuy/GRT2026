package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;

public class ZYXtoXYZeulerConverter {


    /**
     * Convert ZYX (yaw, pitch, roll) → XYZ (roll, pitch, yaw)
     *
     * @param yaw rotation about Z
     * @param pitch rotation about Y
     * @param roll rotation about X
     * @return Euler angles in XYZ order
     */
    public static Rotation3d zyxToXyz(double yaw, double pitch, double roll) {

        double cy = Math.cos(yaw);
        double sy = Math.sin(yaw);
        double cp = Math.cos(pitch);
        double sp = Math.sin(pitch);
        double cr = Math.cos(roll);
        double sr = Math.sin(roll);

        // Build rotation matrix R = Rz * Ry * Rx
        double r00 = cy * cp;
        double r01 = cy * sp * sr - sy * cr;
        double r02 = cy * sp * cr + sy * sr;

        double r10 = sy * cp;
        double r11 = sy * sp * sr + cy * cr;
        double r12 = sy * sp * cr - cy * sr;

        double r20 = -sp;
        double r21 = cp * sr;
        double r22 = cp * cr;

        // Extract XYZ angles
        double x, y, z;

        if (Math.abs(r20) < 1.0) {
            y = Math.asin(-r20);
            x = Math.atan2(r21, r22);
            z = Math.atan2(r10, r00);
        } else {
            // Gimbal lock case
            y = r20 > 0 ? -Math.PI / 2 : Math.PI / 2;
            x = Math.atan2(-r12, r11);
            z = 0; // arbitrary
        }

        return new Rotation3d(x, y, z);
    }


}
