// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;

public class PivotIntakeSubsystem extends SubsystemBase {
    private final TalonFX pivotMotor;
    private final CANcoder canCoder;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final PositionVoltage voltagePosControl = new PositionVoltage(0); // .withEnableFOC(true); enable if re-run with FOC
    private final VoltageOut sysIdVoltage = new VoltageOut(0).withEnableFOC(true);
    private final SysIdRoutine sysIdRoutine;

    // Tunable PID values
    private double kP = IntakeConstants.PIVOT_P;
    private double kI = IntakeConstants.PIVOT_I;
    private double kD = IntakeConstants.PIVOT_D;
    private double kS = IntakeConstants.PIVOT_S;
    private double kV = IntakeConstants.PIVOT_V;
    private double kA = IntakeConstants.PIVOT_A;
    private double cruiseVelocity = IntakeConstants.PIVOT_CRUISE_VELOCITY;
    private double acceleration = IntakeConstants.PIVOT_ACCELERATION;

    public PivotIntakeSubsystem(CANBus canBus) {
        pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, canBus);
        canCoder = new CANcoder(IntakeConstants.PIVOT_CANCODER_ID, canBus);
        configMotors();
        SmartDashboard.putNumber("Intake/Pivot/kP", kP);
        SmartDashboard.putNumber("Intake/Pivot/kI", kI);
        SmartDashboard.putNumber("Intake/Pivot/kD", kD);
        SmartDashboard.putNumber("Intake/Pivot/kV", kV);
        SmartDashboard.putNumber("Intake/Pivot/CruiseVelocity", cruiseVelocity);
        SmartDashboard.putNumber("Intake/Pivot/Acceleration", acceleration);
        SmartDashboard.putNumber("Intake/Pivot/TargetPosition", 0);
        SmartDashboard.putNumber("Intake/Pivot/ManualSpeed", IntakeConstants.MANUAL_PIVOT_SPEED);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds), // ramp rate: 0.5 V/s (slow for limited range)
                Volts.of(4), // step voltage
                Seconds.of(5) // timeout
            ),
            new SysIdRoutine.Mechanism(
                voltage -> pivotMotor.setControl(sysIdVoltage.withOutput(voltage)),
                log -> {
                    log.motor("pivotIntake")
                        .voltage(pivotMotor.getMotorVoltage().getValue())
                        .angularPosition(pivotMotor.getPosition().getValue())
                        .angularVelocity(pivotMotor.getVelocity().getValue());
                },
                this));
    }

    private void configMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID Config
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;

        // Motor Output Config
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Using internal encoder only (FusedCANcoder disabled for sysID)
        config.withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(canCoder.getDeviceID())
            .withRotorToSensorRatio(IntakeConstants.GEAR_RATIO)
            .withSensorToMechanismRatio(1.0));

        // StatorCurrent Limits
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT_ENABLE));

        // Software Limits
        config.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withForwardSoftLimitThreshold(IntakeConstants.PIVOT_FORWARD_LIMIT)
                .withReverseSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(IntakeConstants.PIVOT_REVERSE_LIMIT));

        pivotMotor.getConfigurator().apply(config);
    }


    public void updateTunableValues() {
        double newP = SmartDashboard.getNumber("Intake/Pivot/kP", kP);
        double newI = SmartDashboard.getNumber("Intake/Pivot/kI", kI);
        double newD = SmartDashboard.getNumber("Intake/Pivot/kD", kD);
        double newV = SmartDashboard.getNumber("Intake/Pivot/kV", kV);
        double newCruise = SmartDashboard.getNumber("Intake/Pivot/CruiseVelocity", cruiseVelocity);
        double newAccel = SmartDashboard.getNumber("Intake/Pivot/Acceleration", acceleration);

        // Only reconfig if values changed
        if (newP != kP || newI != kI || newD != kD || newV != kV
            || newCruise != cruiseVelocity || newAccel != acceleration) {
            kP = newP;
            kI = newI;
            kD = newD;
            kV = newV;
            cruiseVelocity = newCruise;
            acceleration = newAccel;
            configMotors();
        }
    }

    public double getAngleDegrees() {
        return pivotMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public double getAbsolutePosition() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Get motor position in rotations
     */
    public double getMotorPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    /**
     * Set pivot position using Velocity Voltage
     * 
     * @param rotations Target position in mechanism rotations
     */
    public void setPosition(double rotations) {
        pivotMotor.setControl(voltagePosControl.withPosition(rotations));
    }

    /**
     * Go to target position from SmartDashboard
     */
    public void goToTunableTarget() {
        double target = SmartDashboard.getNumber("Intake/Pivot/TargetPosition", 0);
        setPosition(target);
    }

    public void zeroEncoder() {
        // canCoder.setPosition(0.0);
        // pivotMotor.setPosition(0.0);
    }

    public void setEncoderToMax() {
        double maxRotations = IntakeConstants.PIVOT_OUT_POS;
        canCoder.setPosition(maxRotations);
        pivotMotor.setPosition(maxRotations);
    }

    /**
     * Manually controls the pivot at a specific speed
     * Software limits will prevent movement past configured bounds
     *
     * @param speed Desired speed from -1.0 to 1.0
     */
    public void setManualSpeed(double speed) {
        pivotMotor.setControl(dutyCycleControl.withOutput(speed));
    }

    public void stop() {
        pivotMotor.setControl(dutyCycleControl.withOutput(0));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        // SmartDashboard!
        SmartDashboard.putNumber("Intake/Pivot/CANcoder/AngleDegrees", getAngleDegrees());
        // SmartDashboard.putNumber("Intake/Pivot/CANcoder/AbsolutePosition", getAbsolutePosition());
        // SmartDashboard.putNumber("Intake/Pivot/CANcoder/Position", canCoder.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Pivot/Motor/DutyCycle", pivotMotor.get());
        SmartDashboard.putNumber("Intake/Pivot/Motor/Position", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot/Motor/Velocity", pivotMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot/Motor/StatorCurrent", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot/Motor/SupplyCurrent", pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot/Motor/AppliedVolts", pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot/Motor/ClosedLoopError", pivotMotor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot/Motor/Temp", pivotMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Pivot/Motor/Connected", pivotMotor.isConnected());
        // SmartDashboard.putBoolean("Intake/Pivot/CANcoder/Connected", canCoder.getPosition().getStatus().isOK());

        SmartDashboard.putNumber("sysIDTest/pivotSetpoint(Rot.)", pivotMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("sysIDTest/pivotPos(Rot.)", pivotMotor.getPosition(false).getValueAsDouble());
    }
}
