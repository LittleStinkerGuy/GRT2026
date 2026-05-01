package frc.robot.util;

public class ComponentStatus {
    public enum MotorControlMode {
        Disabled,
        Follower,
        DutyCycle,
        Voltage,
        TorqueCurrent,
        Position,
        Velocity
    }
}
