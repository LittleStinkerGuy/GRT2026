package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;

public class RollerIntakeSubsystem extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0); // .withEnableFOC(true); enable if re-run with FOC
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut sysIdVoltage = new VoltageOut(0).withEnableFOC(true);
    private final SysIdRoutine sysIdRoutine;

    private double kP = IntakeConstants.ROLLER_P;
    private double kI = IntakeConstants.ROLLER_I;
    private double kD = IntakeConstants.ROLLER_D;
    private double kS = IntakeConstants.ROLLER_S;
    private double kV = IntakeConstants.ROLLER_V;
    private double kA = IntakeConstants.ROLLER_A;

    private double inSpeed = IntakeConstants.ROLLER_IN_SPEED;
    private double outSpeed = IntakeConstants.ROLLER_OUT_SPEED;

    public RollerIntakeSubsystem(CANBus canBus) {
        rollerMotor = new TalonFX(IntakeConstants.ROLLER_CAN_ID, canBus);
        configureMotor();

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds), // ramp rate: 1 V/s
                Volts.of(7), // step voltage
                Seconds.of(10) // timeout
            ),
            new SysIdRoutine.Mechanism(
                voltage -> rollerMotor.setControl(sysIdVoltage.withOutput(voltage)),
                log -> {
                    log.motor("rollerIntake")
                        .voltage(rollerMotor.getMotorVoltage().getValue())
                        .angularPosition(rollerMotor.getPosition().getValue())
                        .angularVelocity(rollerMotor.getVelocity().getValue());
                },
                this));

        SmartDashboard.putNumber("Intake/Roller/kP", kP);
        SmartDashboard.putNumber("Intake/Roller/kI", kI);
        SmartDashboard.putNumber("Intake/Roller/kD", kD);
        SmartDashboard.putNumber("Intake/Roller/kV", kV);
        SmartDashboard.putNumber("Intake/Roller/InSpeed", Math.abs(inSpeed));
        SmartDashboard.putNumber("Intake/Roller/OutSpeed", Math.abs(outSpeed));
        SmartDashboard.putNumber("Intake/Roller/ManualDutyCycle", 1.0);
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
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT)));

        // Velocity PID configuration
        config.withSlot0(new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA));

        rollerMotor.getConfigurator().apply(config);
    }

    public void updateTunableValues() {
        double newP = SmartDashboard.getNumber("Intake/Roller/kP", kP);
        double newI = SmartDashboard.getNumber("Intake/Roller/kI", kI);
        double newD = SmartDashboard.getNumber("Intake/Roller/kD", kD);
        double newV = SmartDashboard.getNumber("Intake/Roller/kV", kV);

        // Only reconfig if values changed
        if (newP != kP || newI != kI || newD != kD || newV != kV) {
            kP = newP;
            kI = newI;
            kD = newD;
            kV = newV;
            configureMotor();
        }
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

        SmartDashboard.putNumber("sysIDTest/rollerSetpoint(RPS)", rollerMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("sysIDTest/rollerVelo(RPS)", rollerMotor.getVelocity(false).getValueAsDouble());
    }

    /**
     * Run intake rollers at specified velocity
     * 
     * @param velocity target velocity in rotations per second
     */
    public void setVelocity(double velocity) {
        rollerMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    /**
     * Run intake at specified duty cycle (for simple testing)
     * 
     * @param dutyCycle -1.0 to 1.0
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

    /**
     * Run intake in (intaking direction) at tunable speed
     */
    public void runIn() {
        inSpeed = SmartDashboard.getNumber("Intake/Roller/InSpeed", Math.abs(IntakeConstants.ROLLER_IN_SPEED));
        setVelocity(-inSpeed); // Negative for intaking based on constants
    }

    /**
     * Run intake out (ejecting direction) at tunable speed
     */
    public void runOut() {
        outSpeed = SmartDashboard.getNumber("Intake/Roller/OutSpeed", Math.abs(IntakeConstants.ROLLER_OUT_SPEED));
        setVelocity(outSpeed);
    }

    /**
     * Run intake in at tunable duty cycle (simpler control for testing)
     */
    public void runInDutyCycle() {
        double dc = SmartDashboard.getNumber("Intake/Roller/ManualDutyCycle", 1.0);
        setDutyCycle(-dc);
    }

    /**
     * Run intake out at tunable duty cycle (simpler control for testing)
     */
    public void runOutDutyCycle() {
        double dc = SmartDashboard.getNumber("Intake/Roller/ManualDutyCycle", 1.0);
        setDutyCycle(dc);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Check if the roller is currently running
     */
    public boolean isRunning() {
        return Math.abs(rollerMotor.get()) > 0.01;
    }
}
