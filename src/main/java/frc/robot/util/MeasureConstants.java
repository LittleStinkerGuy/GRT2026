package frc.robot.util;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public final class MeasureConstants {
    public static final Voltage ZERO_VOLTAGE = Volts.zero();
    public static final AngularVelocity ZERO_ANGULAR_VELOCITY = RotationsPerSecond.zero();
    public static final Angle ZERO_ANGLE = Rotations.zero();

    private MeasureConstants() {}
}
