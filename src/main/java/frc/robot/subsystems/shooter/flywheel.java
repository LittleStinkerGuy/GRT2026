package frc.robot.subsystems.shooter;

import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import org.littletonrobotics.junction.Logger;

public class flywheel extends SubsystemBase {

    private final TalonFX upperMotor;
    private final DutyCycleOut dutyCycl = new DutyCycleOut(0);

    private double commandedDutyCycle = 0.0;
    private static final String LOG_PREFIX = "FlyWheel/";

    public flywheel(CANBus cn) {
        // Construct motors directly on the CAN bus
        upperMotor = new TalonFX(railgunConstants.upperId, cn);
        config();
       
    }

    public void config(){
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.Feedback.SensorToMechanismRatio = railgunConstants.gearRatioUpper;
        //CurrentLimitsConfigs currLim = new CurrentLimitsConfigs().withStatorCurrentLimit(40.0).withStatorCurrentLimitEnable(true);
        //cfg.withCurrentLimits(currLim);
        upperMotor.getConfigurator().apply(cfg);
       
    }

    public void flySpeed(double speed){
        commandedDutyCycle = speed;
        upperMotor.setControl(dutyCycl.withOutput(speed));
    }

    @Override
    public void periodic(){
        sendData();
        
    }

    public void sendData(){
        Logger.recordOutput(LOG_PREFIX + "PositionRotations",
            upperMotor.getPosition().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "VelocityRPS",
            upperMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "VelocityRPM",
           (60* upperMotor.getVelocity().getValueAsDouble()));

        Logger.recordOutput(LOG_PREFIX + "AppliedVolts",
            upperMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "SupplyVoltage",
            upperMotor.getSupplyVoltage().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "StatorCurrentAmps",
            upperMotor.getStatorCurrent().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "SupplyCurrentAmps",
            upperMotor.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "TemperatureC",
            upperMotor.getDeviceTemp().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "CommandedDutyCycle",
            commandedDutyCycle);

        Logger.recordOutput(LOG_PREFIX + "Connected",
            upperMotor.isConnected());

        Logger.recordOutput(LOG_PREFIX + "RPS", upperMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "Linear_Velocity_mPs", upperMotor.getVelocity().getValueAsDouble()*0.0762/2);
    }
}
