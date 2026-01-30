package frc.robot.subsystems.shooter;


import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class hood extends SubsystemBase {

    private final TalonFX hoodMotor;
    private final DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private CANdi limit;

    public hood(CANBus cn) {
        // Construct motors directly on the CAN bus
        hoodMotor = new TalonFX(railgunConstants.hoodId, cn);
        limit = new CANdi(railgunConstants.limitId, cn);

        // Initialize hood to starting angle
        hoodMotor.setPosition(railgunConstants.initHoodAngle);
        config();
    }

    public void config(){
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        CurrentLimitsConfigs currLim = new CurrentLimitsConfigs().withStatorCurrentLimit(50.0).withStatorCurrentLimitEnable(true);
        cfg.withCurrentLimits(currLim);
        hoodMotor.getConfigurator().apply(cfg);
    }

    public void hoodSpeed(double speed){
        if(hoodMotor.getPosition().getValueAsDouble() >= railgunConstants.initHoodAngle && speed >0){
            hoodMotor.setControl(dutyCycl.withOutput(0));
        }else if(hoodMotor.getPosition().getValueAsDouble() <= railgunConstants.lowerAngle && speed <0){
            hoodMotor.setControl(dutyCycl.withOutput(0));
        }else{
        hoodMotor.setControl(dutyCycl.withOutput(speed));
        }
    }

    @Override
    public void periodic(){
        if(limit.getS1Closed().refresh().getValue()){
            hoodMotor.setPosition(railgunConstants.initHoodAngle);
        }
    }
}