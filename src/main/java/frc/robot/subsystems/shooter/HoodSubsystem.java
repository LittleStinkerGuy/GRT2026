package frc.robot.subsystems.shooter;


import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX hoodMotor;
    private final DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private CANdi limit;

    private double commandedDutyCycle = 0.0;
    private static final String LOG_PREFIX = "Shooter/Hood/";

    public HoodSubsystem(CANBus cn) {
        // Construct motors directly on the CAN bus
        hoodMotor = new TalonFX(ShooterConstants.HOOD_CAN_ID, cn);
        limit = new CANdi(ShooterConstants.FLYWHEEL_ENCODER_ID, cn);

        // Initialize hood to starting angle
        hoodMotor.setPosition(ShooterConstants.INIT_HOOD_ANGLE);
        config();
    }

    public void config(){
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO_HOOD;
        CurrentLimitsConfigs currLim = new CurrentLimitsConfigs().withStatorCurrentLimit(50.0).withStatorCurrentLimitEnable(true);
        cfg.withCurrentLimits(currLim);
        hoodMotor.getConfigurator().apply(cfg);
        cfg.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(ShooterConstants.UPPER_ANGLE)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ShooterConstants.LOWER_ANGLE)
            );
    }

    public void hoodSpeed(double speed){
        
       /*  if(hoodMotor.getPosition().getValueAsDouble() >= ShooterConstants.INIT_HOOD_ANGLE && speed >0){
            hoodMotor.setControl(dutyCycl.withOutput(0));
            commandedDutyCycle = 0;
        }else if(hoodMotor.getPosition().getValueAsDouble() <= ShooterConstants.LOWER_ANGLE && speed <0){
            hoodMotor.setControl(dutyCycl.withOutput(0));
            commandedDutyCycle = 0;
        }else{
            hoodMotor.setControl(dutyCycl.withOutput(speed));
            commandedDutyCycle = speed;
        }
            */
            commandedDutyCycle = speed;
            hoodMotor.setControl(dutyCycl.withOutput(speed));
        
    }

    boolean prevPress = false;
    @Override
    public void periodic(){
        if(limit.getS1Closed().refresh().getValue() && !prevPress){
            hoodMotor.setPosition(ShooterConstants.INIT_HOOD_ANGLE);
            prevPress = true;
        }
        
        if(!limit.getS1Closed().refresh().getValue()){
            prevPress = false;
        }
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
