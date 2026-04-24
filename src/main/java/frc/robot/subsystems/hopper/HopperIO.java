package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        public Angle position;
        public AngularVelocity velocity;
        public AngularAcceleration acceleration;
        public Voltage appliedVoltage;
        public Current supplyCurrent;
        public Current torqueCurrent;
        public Current statorCurrent;
        public Temperature temp;
        public boolean tempFault;
        public boolean connected;
    }

    default void updateInputs(HopperIOInputs inputs) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setVelocity(AngularVelocity velocity) {}

    default void stop() {}

}
