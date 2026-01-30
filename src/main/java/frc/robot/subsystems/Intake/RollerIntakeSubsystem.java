package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.Amps;
import com.ctre.phoenix6.CANBus;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class RollerIntakeSubsystem extends SubsystemBase {
    private TalonFX rollerMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    public RollerIntakeSubsystem(CANBus canBus) {
        rollerMotor = new TalonFX(IntakeConstants.ROLLER_CAN_ID, Constants.CAN_BUS);
        configureMotor();
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output
        config.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(IntakeConstants.ROLLER_INVERTED));

        // Current limits
        config.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(false)
                        .withStatorCurrentLimit(Amps.of(IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT))
        );

        config.withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(IntakeConstants.ROLLER_OPEN_LOOP_RAMP)
        );

        rollerMotor.getConfigurator().apply(config);
    }
    

    @Override
    public void periodic() {
        // Motor status logging!
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
