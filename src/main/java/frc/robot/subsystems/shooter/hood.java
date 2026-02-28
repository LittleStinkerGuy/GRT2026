package frc.robot.subsystems.shooter;

import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import org.littletonrobotics.junction.Logger;

public class hood extends SubsystemBase {

    private final TalonFX hoodMotor;
    private final DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private PositionVoltage focThing = new PositionVoltage(0);
    //private final CANcoder hoodCoder;

    private double wantedAngle = 0;

    private double commandedDutyCycle = 0.0;
    private static final String LOG_PREFIX = "Hood/";

    public hood(CANBus cn) {
        hoodMotor = new TalonFX(railgunConstants.hoodId, cn);
        //hoodCoder = new CANcoder(railgunConstants.hoodEncoderId, cn);
        config();
    }

    public void config(){
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CurrentLimitsConfigs currLim = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);;
        cfg.withCurrentLimits(currLim);
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = railgunConstants.upperAngle;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = railgunConstants.lowerAngle;
        cfg.Feedback.SensorToMechanismRatio = 1;

        cfg.Slot0.kP = 8;
        cfg.Slot0.kI = 3;

        /* 
        CANcoderConfiguration ccfg = new CANcoderConfiguration();
        ccfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; 

        hoodCoder.getConfigurator().apply(ccfg);

        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = railgunConstants.hoodEncoderId;
        */
        cfg.Feedback.SensorToMechanismRatio = railgunConstants.gearRatioHood;

        hoodMotor.getConfigurator().apply(cfg);
    }

    public void setHoodAngle(double rotationAngle){
        wantedAngle = rotationAngle;
        // if(rotationAngle >= railgunConstants.lowerAngle && rotationAngle <= railgunConstants.upperAngle){
            hoodMotor.setControl(focThing.withPosition(rotationAngle));
            System.out.println("HoodControl" + rotationAngle);
        // }
    }

    public boolean wantedAngl(){
        if(Math.abs(wantedAngle-hoodMotor.getPosition().getValueAsDouble()) <0.5){
            return true;
        }else{
            return false;
        }
    }

    public void hoodSpeed(double speed){
        commandedDutyCycle = speed;
        
        double pos = hoodMotor.getPosition().refresh().getValueAsDouble();
        if(pos >= railgunConstants.upperAngle && speed >0){
            hoodMotor.setControl(dutyCycl.withOutput(0));
        }else if(pos <= railgunConstants.lowerAngle && speed <0){
            hoodMotor.setControl(dutyCycl.withOutput(0));
        }else{
            hoodMotor.setControl(dutyCycl.withOutput(speed));
        }
        
    }

    public double getPos(){
        return hoodMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic(){
        sendData();   
    }

    public void sendData(){
        Logger.recordOutput(LOG_PREFIX + "PositionRotations",
            hoodMotor.getPosition().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "VelocityRPS",
            hoodMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "AppliedVolts",
            hoodMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "SupplyVoltage",
            hoodMotor.getSupplyVoltage().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "StatorCurrentAmps",
            hoodMotor.getStatorCurrent().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "SupplyCurrentAmps",
            hoodMotor.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "TemperatureC",
            hoodMotor.getDeviceTemp().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "CommandedDutyCycle",
            commandedDutyCycle);

        Logger.recordOutput(LOG_PREFIX + "Connected",
            hoodMotor.isConnected());
    }


}