package frc.robot.subsystems.shooter;

import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class flywheel extends SubsystemBase {

    private final TalonFX upperMotor;
    private final DutyCycleOut dutyCycl = new DutyCycleOut(0);

    public flywheel(CANBus cn) {
        // Construct motors directly on the CAN bus
        upperMotor = new TalonFX(railgunConstants.upperId, cn);
       
    }

    public void flySpeed(double speed){
        upperMotor.setControl(dutyCycl.withOutput(speed));
    }
}