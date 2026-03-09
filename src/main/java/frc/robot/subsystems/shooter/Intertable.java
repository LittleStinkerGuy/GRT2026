package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Intertable {

    private static final Intertable INSTANCE = new Intertable();

    private static InterpolatingDoubleTreeMap shooterAngle = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap shooterRPS = new InterpolatingDoubleTreeMap();

    private static boolean initialized = false;

    private Intertable() {
        if (!initialized) {
            // Hood angles must be within limits: 0.06 to 0.169 rotations
            // Closer distance = higher angle, farther = lower angle
            shooterAngle.put(1.524, 0.16); // closest - highest angle
            shooterAngle.put(1.651, 0.14);
            shooterAngle.put(1.981, 0.12);
            shooterAngle.put(2.261, 0.10);
            shooterAngle.put(2.616, 0.08); // farthest - lowest angle

            shooterRPS.put(1.524, 45.0);
            shooterRPS.put(1.651, 45.0);
            shooterRPS.put(1.981, 50.0);
            shooterRPS.put(2.261, 50.0);
            shooterRPS.put(2.616, 60.0);

            initialized = true;
        }
    }

    public static Intertable getInstance() {
        return INSTANCE;
    }

    public double getRPS(double dis) {
        return shooterRPS.get(dis);
    }

    public double getAngle(double dis) {
        return shooterAngle.get(dis);
    }
}
