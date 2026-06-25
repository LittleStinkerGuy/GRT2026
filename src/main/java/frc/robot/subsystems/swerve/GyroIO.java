package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

    public static class GyroIOInputs {
        public double yawPositionRot = 0.0;
        public double yawVelocityRPS = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

        public boolean connected = false;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void resetHeading() {}
}
