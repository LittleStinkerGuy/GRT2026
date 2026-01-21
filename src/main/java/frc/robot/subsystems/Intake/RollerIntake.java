package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        rollerMotor = new TalonFX(IntakeConstants.ROLLER_CAN_ID, IntakeConstants.ROLLER_CAN_BUS);
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    @Override
    public void periodic() {
        
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
}
