package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterLearner {

    private static final double RPM_STEP = 5.0;
    private static final double ANGLE_STEP = 0.005;

    private final DoublePublisher rpmOffsetPub;
    private final DoubleSubscriber rpmOffsetSub;
    private final DoublePublisher hoodOffsetPub;
    private final DoubleSubscriber hoodOffsetSub;

    public ShooterLearner() {
        NetworkTable t = NetworkTableInstance.getDefault().getTable("ShooterLearner");
        rpmOffsetPub = t.getDoubleTopic("rpmOffset").publish();
        rpmOffsetSub = t.getDoubleTopic("rpmOffset").subscribe(0.0);
        hoodOffsetPub = t.getDoubleTopic("hoodOffset").publish();
        hoodOffsetSub = t.getDoubleTopic("hoodOffset").subscribe(0.0);
        rpmOffsetPub.set(0);
        hoodOffsetPub.set(0);
    }

    public double getRPM(double baseRPM) {
        return baseRPM + rpmOffsetSub.get();
    }

    public double getHoodAngle(double baseAngle) {
        return baseAngle + hoodOffsetSub.get();
    }

    public void rpmUp() {
        rpmOffsetPub.set(rpmOffsetSub.get() + RPM_STEP);
    }

    public void rpmDown() {
        rpmOffsetPub.set(rpmOffsetSub.get() - RPM_STEP);
    }

    public void hoodUp() {
        hoodOffsetPub.set(hoodOffsetSub.get() + ANGLE_STEP);
    }

    public void hoodDown() {
        hoodOffsetPub.set(hoodOffsetSub.get() - ANGLE_STEP);
    }

    public void reset() {
        rpmOffsetPub.set(0);
        hoodOffsetPub.set(0);
    }

    /** Print the current calibration point so it can be copied into Intertable.java. */
    public void log(double distance, double rpm, double angle) {
        System.out.printf("[shooterLearner] d=%.3f rpm=%.2f angle=%.4f%n", distance, rpm, angle);
    }
}
