package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HopperConstants;


public class HopperMotor extends SubsystemBase {

    private final TalonFX krakenMotor;
    private final VelocityVoltage velocityControl;
    private final DutyCycleOut dutyCycleControl;
    
    public HopperMotor() {
        krakenMotor = new TalonFX(HopperConstants.KRAKEN_CAN_ID, "can");
        velocityControl = new VelocityVoltage(0);
        dutyCycleControl = new DutyCycleOut(0);

        configureMotor();
    }
    
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12; 
        
        krakenMotor.getConfigurator().apply(config);
    }
    

    public void spinAtTargetRPM() {
        // sorry for not using units library daniel
        double rotationsPerSecond = HopperConstants.TARGET_RPM / 60.0;
        krakenMotor.setControl(velocityControl.withVelocity(rotationsPerSecond));
    }
    

    public void spinAtRPM(double rpm) {
        double rotationsPerSecond = rpm / 60.0;
        krakenMotor.setControl(velocityControl.withVelocity(rotationsPerSecond));
    }
  
    public void setManualControl(double percentOutput) {
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));
        krakenMotor.setControl(dutyCycleControl.withOutput(percentOutput));
    }
    
  
    public void stop() {
        krakenMotor.setControl(dutyCycleControl.withOutput(0));
    }
    
  
    public double getCurrentRPM() {
        return krakenMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getMotorOutput() {
        return krakenMotor.get();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper/CurrentRPM", getCurrentRPM());
        SmartDashboard.putNumber("Hopper/MotorOutput", getMotorOutput());
        SmartDashboard.putNumber("Hopper/Velocity", krakenMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Hopper/Current", krakenMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hopper/Voltage", krakenMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hopper/Temp", krakenMotor.getDeviceTemp().getValueAsDouble());
    }
}