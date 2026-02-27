package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.Amps;
import com.ctre.phoenix6.CANBus;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class RollerIntakeSubsystem extends SubsystemBase {
    private TalonFX rollerMotor;
    private final VelocityTorqueCurrentFOC velocityFOCRequest = new VelocityTorqueCurrentFOC(0);

    private TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    private double inSpeed = IntakeConstants.ROLLER_IN_SPEED;
    private double outSpeed = IntakeConstants.ROLLER_OUT_SPEED;

    public RollerIntakeSubsystem(CANBus canBus) {
        rollerMotor = new TalonFX(IntakeConstants.ROLLER_CAN_ID, canBus);
        configureMotor();
        rollerMotor.getConfigurator().apply(rollerConfig);

        // Initialize tunable speeds in NetworkTables
        SmartDashboard.putNumber("Intake/Roller/InSpeed", inSpeed);
        SmartDashboard.putNumber("Intake/Roller/OutSpeed", Math.abs(outSpeed));
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

        // Velocity PID configuration
        config.withSlot0(new Slot0Configs()
                .withKP(IntakeConstants.ROLLER_P)
                .withKI(IntakeConstants.ROLLER_I)
                .withKD(IntakeConstants.ROLLER_D)
                .withKV(IntakeConstants.ROLLER_V)
        );

        rollerMotor.getConfigurator().apply(config);
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Roller/DutyCycle", rollerMotor.get());
        SmartDashboard.putNumber("Intake/Roller/Position", rollerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller/Velocity", rollerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller/StatorCurrent", rollerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller/SupplyCurrent", rollerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller/AppliedVolts", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller/SupplyVoltage", rollerMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller/Temp", rollerMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Roller/Connected", rollerMotor.isConnected());
        SmartDashboard.putBoolean("Intake/Roller/IsRunning", Math.abs(rollerMotor.get()) > 0.01);
    }

    /**
     * Run intake rollers at specified velocity
     * @param velocity target velocity in rotations per second
     */
    public void setVelocity(double velocity) {
        rollerMotor.setControl(velocityFOCRequest.withVelocity(velocity));
    }

    /**
     * Stop the intake rollers
     */
    public void stop() {
        rollerMotor.setControl(velocityFOCRequest.withVelocity(0));
    }

    /**
     * Run intake in (positive direction) at tunable speed
     */
    public void runIn() {
        inSpeed = SmartDashboard.getNumber("Intake/Roller/InSpeed", IntakeConstants.ROLLER_IN_SPEED);
        setVelocity(inSpeed);
    }

    /**
     * Run intake out (negative direction) at tunable speed
     */
    public void runOut() {
        outSpeed = SmartDashboard.getNumber("Intake/Roller/OutSpeed", Math.abs(IntakeConstants.ROLLER_OUT_SPEED));
        setVelocity(-outSpeed);
    }

    //Check if the roller is currently running

    public boolean isRunning() {
        return Math.abs(rollerMotor.get()) > 0.01;
    }
}
