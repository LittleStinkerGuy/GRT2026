package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {

    public static class GyroIOInputs {
        public Angle yawPosition = Rotations.of(0.0);
        public AngularVelocity yawVelocity = RotationsPerSecond.of(0.0);
        public double[] odometryYawTimestamps = new double[] {};
        public Angle[] odometryYawPositions = new Angle[] {};

        public boolean connected = false;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void resetHeading() {}
}
