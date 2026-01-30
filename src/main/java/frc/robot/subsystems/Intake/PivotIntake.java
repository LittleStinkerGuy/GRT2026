// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class PivotIntake extends SubsystemBase {
  private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, Constants.CAN_BUS);

  private final CANdle candle = new CANdle(IntakeConstants.CANDLE_ID);

  private final DigitalInput bottomLimitSwitch = new DigitalInput(IntakeConstants.BOTTOM_LIMIT_SWITCH_DIO);
  private final DigitalInput topLimitSwitch = new DigitalInput(IntakeConstants.TOP_LIMIT_SWITCH_DIO);

  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

  // --- Position control (commented out for now) ---
  // private final PositionVoltage positionControl = new PositionVoltage(0);
  // private final CANcoder canCoder = new CANcoder(IntakeConstants.ENCODER_ID);
  // private boolean previousBottomLimitState = false;
  // private boolean previousTopLimitState = false;

  public PivotIntake() {
    configCANdle();
    configMotors();
    // configEncoder();
  }

  private void configMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // --- PID config (commented out for now) ---
    // config.Slot0.kP = IntakeConstants.PIVOT_P;
    // config.Slot0.kI = IntakeConstants.PIVOT_I;
    // config.Slot0.kD = IntakeConstants.PIVOT_D;
    // config.Slot0.kV = IntakeConstants.PIVOT_F;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Configure current limits
    config.CurrentLimits
        .withStatorCurrentLimit(40)
        .withStatorCurrentLimitEnable(true);

    pivotMotor.getConfigurator().apply(config);
  }

  // private void configEncoder() {
  //   CANcoderConfiguration config = new CANcoderConfiguration();
  //   canCoder.getConfigurator().apply(config);
  // }

  private void configCANdle() {
    CANdleConfiguration config = new CANdleConfiguration();
    candle.getConfigurator().apply(config);
  }

  // --- Position control methods (commented out for now) ---
  // public double getAngleDegrees() {
  //   return canCoder.getPosition().getValueAsDouble() * 360.0;
  // }
  //
  // public double getAbsolutePosition() {
  //   return canCoder.getAbsolutePosition().getValueAsDouble();
  // }
  //
  // public void setAngle(double angleDegrees) {
  //   angleDegrees = Math.max(IntakeConstants.STOWED_POS,
  //                          Math.min(IntakeConstants.EXTENDED_POS, angleDegrees));
  //   double motorRotations = (angleDegrees / 360.0) / IntakeConstants.GEAR_RATIO;
  //   pivotMotor.setControl(positionControl.withPosition(motorRotations));
  // }
  //
  // public void zeroEncoder() {
  //   canCoder.setPosition(0.0);
  // }
  //
  // public void setEncoderToMax() {
  //   double maxRotations = IntakeConstants.EXTENDED_POS / 360.0;
  //   canCoder.setPosition(maxRotations);
  // }

  public boolean isAtTopLimit() {
    return !topLimitSwitch.get();
  }

  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  /**
   * Manually controls the pivot at a specific speed
   * Prevents pivot from moving past top & bottom limits
   *
   * @param speed Desired speed from -1.0 to 1.0
   */
  public void setManualSpeed(double speed) {
    if (isAtTopLimit() && speed > 0) {
      speed = 0;
    }
    if (isAtBottomLimit() && speed < 0) {
      speed = 0;
    }
    pivotMotor.setControl(dutyCycleControl.withOutput(speed));
  }

  public void stop() {
    pivotMotor.setControl(dutyCycleControl.withOutput(0));
  }

  @Override
  public void periodic() {
    boolean topLimit = isAtTopLimit();
    boolean bottomLimit = isAtBottomLimit();

    // --- Encoder reset on limit switch (commented out for now) ---
    // if (bottomLimit && !previousBottomLimitState) {
    //   zeroEncoder();
    // }
    // previousBottomLimitState = bottomLimit;
    //
    // if (topLimit && !previousTopLimitState) {
    //   setEncoderToMax();
    // }
    // previousTopLimitState = topLimit;

    // SmartDashboard
    // SmartDashboard.putNumber("Intake/Pivot/AngleDegrees", getAngleDegrees());
    // SmartDashboard.putNumber("Intake/Pivot/AbsolutePosition", getAbsolutePosition());

    SmartDashboard.putBoolean("Intake/Pivot/AtTopLimit", topLimit);
    SmartDashboard.putBoolean("Intake/Pivot/AtBottomLimit", bottomLimit);

    SmartDashboard.putNumber("Intake/Pivot/DutyCycle", pivotMotor.get());
    SmartDashboard.putNumber("Intake/Pivot/Velocity", pivotMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Current", pivotMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/MotorTemp", pivotMotor.getDeviceTemp().getValueAsDouble());
  }
}
