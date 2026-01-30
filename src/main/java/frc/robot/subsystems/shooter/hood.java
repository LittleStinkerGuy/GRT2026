package frc.robot.subsystems.shooter;


import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;

public class hood extends SubsystemBase {

    private final TalonFX hoodMotor;

    public hood(CANBus cn) {
        // Construct motors directly on the CAN bus
        hoodMotor = new TalonFX(railgunConstants.hoodId, cn);

        // Initialize hood to starting angle
        hoodMotor.setPosition(railgunConstants.initHoodAngle);
    }

    public void hoodSpeed(double speed){
        hoodMotor.set(speed);
    }
}