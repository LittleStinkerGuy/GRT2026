package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
// ig im using units library now yw daniel!
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;


public class HopperSubsystem extends SubsystemBase {

    private final TalonFX krakenMotor;
    // private final VelocityVoltage velocityControl;
    private final DutyCycleOut dutyCycleControl;

    private double manualSpeed = HopperConstants.MANUAL_SPEED;

    public HopperSubsystem(CANBus canBus) {
        krakenMotor = new TalonFX(HopperConstants.KRAKEN_CAN_ID, canBus);
        // velocityControl = new VelocityVoltage(0);
        dutyCycleControl = new DutyCycleOut(0);

        configureMotor();

        // // Initialize tunable speed in NetworkTables
        // SmartDashboard.putNumber("Hopper/ManualSpeed", manualSpeed);
    }


    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output
        config.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(HopperConstants.HOPPERINVERTED));

        // Current limits
        config.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(HopperConstants.STATOR_CURRENT_LIMIT_ENABLE)
                        .withStatorCurrentLimit(Amps.of(HopperConstants.STATOR_CURRENT_LIMIT_AMPS))
        );

        config.withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(HopperConstants.DUTY_CYCLE_OPEN_LOOP_RAMP)
        );

        krakenMotor.getConfigurator().apply(config);
    }
    

    // --- RPM control methods (commented out for now) ---
    // public void spinAtTargetRPM() {
    //     double rotationsPerSecond = HopperConstants.TARGET_RPM / 60.0;
    //     krakenMotor.setControl(velocityControl.withVelocity(rotationsPerSecond));
    // }
    //
    // public void spinAtRPM(double rpm) {
    //     double rotationsPerSecond = rpm / 60.0;
    //     krakenMotor.setControl(velocityControl.withVelocity(rotationsPerSecond));
    // }
    //
    // public double getCurrentRPM() {
    //     return krakenMotor.getVelocity().getValueAsDouble() * 60.0;
    // }

    public void setManualControl(double percentOutput) {
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));
        krakenMotor.setControl(dutyCycleControl.withOutput(percentOutput));
    }

    public void stop() {
        dutyCycleControl.withOutput(0);
        krakenMotor.setControl(dutyCycleControl);
    }

    /**
     * Run hopper forward at tunable speed
     */
    public void runForward() {
        // manualSpeed = SmartDashboard.getNumber("Hopper/ManualSpeed", HopperConstants.MANUAL_SPEED);
        setManualControl(manualSpeed);
    }

    /**
     * Run hopper reverse at tunable speed
     */
    public void runReverse() {
        // manualSpeed = SmartDashboard.getNumber("Hopper/ManualSpeed", HopperConstants.MANUAL_SPEED);
        setManualControl(-manualSpeed);
    }

    public double getMotorOutput() {
        return krakenMotor.get();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Hopper/MotorOutput", getMotorOutput());
        // SmartDashboard.putNumber("Hopper/Position", krakenMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Hopper/Velocity", krakenMotor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Hopper/StatorCurrent", krakenMotor.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Hopper/SupplyCurrent", krakenMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Hopper/AppliedVolts", krakenMotor.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Hopper/SupplyVoltage", krakenMotor.getSupplyVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Hopper/Temp", krakenMotor.getDeviceTemp().getValueAsDouble());
        // SmartDashboard.putBoolean("Hopper/Connected", krakenMotor.isConnected());
    }
}
