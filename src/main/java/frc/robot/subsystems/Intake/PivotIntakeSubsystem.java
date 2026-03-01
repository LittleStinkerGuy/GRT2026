// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.CANBus;

public class PivotIntakeSubsystem extends SubsystemBase {
  private final TalonFX pivotMotor;
  private final CANcoder canCoder;
  private final CANdle candle;

  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
// private final PositionVoltage positionControl = new PositionVoltage(0);
  private final MotionMagicTorqueCurrentFOC motionMagicControl = new MotionMagicTorqueCurrentFOC(0);

  public PivotIntakeSubsystem(CANBus canBus) {
    pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, canBus);
    canCoder = new CANcoder(IntakeConstants.PIVOT_CANCODER_ID, canBus);
    candle = new CANdle(IntakeConstants.CANDLE_ID);
    configEncoder();
    configCANdle();
    configMotors();
  }

  private void configMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID Config
    config.Slot0.kP = IntakeConstants.PIVOT_P;
    config.Slot0.kI = IntakeConstants.PIVOT_I;
    config.Slot0.kD = IntakeConstants.PIVOT_D;
    config.Slot0.kV = IntakeConstants.PIVOT_F;
    // Motor Output Config
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // FOCTorqueCurrent Limits
    config.TorqueCurrent.PeakForwardTorqueCurrent = IntakeConstants.PIVOT_PEAK_TORQUE_CURRENT;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -IntakeConstants.PIVOT_PEAK_TORQUE_CURRENT;
    // StatorCurrent Limits
    config.withCurrentLimits(
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT)
          .withStatorCurrentLimitEnable(IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT_ENABLE)
    );
    // Motion Magic Config
    config.withMotionMagic(
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(IntakeConstants.PIVOT_CRUISE_VELOCITY)
          .withMotionMagicAcceleration(IntakeConstants.PIVOT_ACCELERATION)
    );
    // Software Limits
    config.withSoftwareLimitSwitch(
      new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(IntakeConstants.TOP_LIMIT)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(IntakeConstants.BOTTOM_LIMIT)
    );

    pivotMotor.getConfigurator().apply(config);
  }

  private void configEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    canCoder.getConfigurator().apply(config);
  }

  private void configCANdle() {
    CANdleConfiguration config = new CANdleConfiguration();
    candle.getConfigurator().apply(config);
  }

  public double getAngleDegrees() {
    return canCoder.getPosition().getValueAsDouble() * 360.0;
  }

  public double getAbsolutePosition() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Set pivot angle using Motion Magic with FOC torque current control
   * @param angleDegrees Target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    angleDegrees = Math.max(IntakeConstants.PIVOT_IN_POS,
                           Math.min(IntakeConstants.PIVOT_OUT_POS, angleDegrees));
    double motorRotations = (angleDegrees / 360.0) / IntakeConstants.GEAR_RATIO;
    pivotMotor.setControl(motionMagicControl.withPosition(motorRotations));
  }

  /**
   * Set pivot position using Motion Magic with FOC torque current control
   * @param rotations Target position in rotations
   */
  public void setPosition(double rotations) {
    pivotMotor.setControl(motionMagicControl.withPosition(rotations));
  }

  public void zeroEncoder() {
    canCoder.setPosition(0.0);
  }

  public void setEncoderToMax() {
    double maxRotations = IntakeConstants.PIVOT_OUT_POS / 360.0;
    canCoder.setPosition(maxRotations);
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

  @Override
  public void periodic() {
    // SmartDashboard
    SmartDashboard.putNumber("Intake/Pivot/AngleDegrees", getAngleDegrees());
    SmartDashboard.putNumber("Intake/Pivot/AbsolutePosition", getAbsolutePosition());

    SmartDashboard.putNumber("Intake/Pivot/DutyCycle", pivotMotor.get());
    SmartDashboard.putNumber("Intake/Pivot/Position", pivotMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Velocity", pivotMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/StatorCurrent", pivotMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/SupplyCurrent", pivotMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/AppliedVolts", pivotMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/SupplyVoltage", pivotMotor.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Temp", pivotMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putBoolean("Intake/Pivot/Connected", pivotMotor.isConnected());
  }
}
