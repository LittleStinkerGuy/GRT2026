package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class RollerIntake extends SubsystemBase {
    private TalonFX rollerMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    private TalonFXConfiguration rollerConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
        );

    public RollerIntake() {
        rollerMotor = new TalonFX(IntakeConstants.ROLLER_CAN_ID, "can");
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    @Override
    public void periodic() {
        // Motor status
        SmartDashboard.putNumber("Intake/Roller/DutyCycle", rollerMotor.get());
        SmartDashboard.putNumber("Intake/Roller/Velocity", rollerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller/Current", rollerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller/Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller/MotorTemp", rollerMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Roller/IsRunning", Math.abs(rollerMotor.get()) > 0.01);
    }

    /**
     * Run intake rollers at specified duty cycle (percent output)
     * @param dutyCycle value between -1.0 and 1.0
     */
    public void setDutyCycle(double dutyCycle) {
        rollerMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    /**
     * Stop the intake rollers
     */
    public void stop() {
        rollerMotor.setControl(dutyCycleRequest.withOutput(0));
    }

    //Check if the roller is currently running

    public boolean isRunning() {
        return Math.abs(rollerMotor.get()) > 0.01;
    }
}
