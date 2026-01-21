// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class PivotIntake extends SubsystemBase {
  // Kraken motors
  private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, "can");

  // CANdle
  private final CANdle candle = new CANdle(IntakeConstants.CANDLE_ID);

  // Limit switches
  private final DigitalInput bottomLimitSwitch = new DigitalInput(IntakeConstants.MIN_LIMIT_SWITCH_ID_DIO);
  private final DigitalInput topLimitSwitch = new DigitalInput(IntakeConstants.MAX_LIMIT_SWITCH_ID_DIO);

  // Encoder
  private final CANcoder canCoder = new CANcoder(IntakeConstants.ENCODER_ID);
  
  // Duty cycle (percent output) control
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

  //Position based voltage control with PID 
  private final PositionVoltage positionControl = new PositionVoltage(0);

  // Track previous limit switch states
  private boolean previousBottomLimitState = false;
  private boolean previousTopLimitState = false;

  public PivotIntake() {
    configCANdle();
    configMotors();
    configEncoder();
  }

  
  


  /**
   * Configures both pivot motors with PID, brake mode, & follower setup.
   * The left motor is the leader, and the right motor follows AFTER being inversed because they are 
   * directly positioned opposite to each other.
   */
  private void configMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID Config for position control 
    // I know I'm just coding manual atm but like surely we'll have stowed / extended positions eventually
    config.Slot0.kP = IntakeConstants.PIVOT_P;
    config.Slot0.kI = IntakeConstants.PIVOT_I;
    config.Slot0.kD = IntakeConstants.PIVOT_D;
    config.Slot0.kV = IntakeConstants.PIVOT_F; // Feedforward gain (accounts for forces)

    // Break mode!
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  }

  //  Config encoder
  private void configEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    canCoder.getConfigurator().apply(config);
  }

  //  Config CANdle
  private void configCANdle() {
    CANdleConfiguration config = new CANdleConfiguration();
    candle.getConfigurator().apply(config);
  }
   /**
   * Converts encoder rotations to degrees by multiplying by 360
   * @return Current angle in degrees
   */
  public double getAngleDegrees() {
    return canCoder.getPosition().getValueAsDouble() * 360.0;
  }


  /**
   * @return Absolute encoder position in rotations (0.0 to 1.0)
   */
  public double getAbsolutePosition() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean isAtTopLimit() {
    // Prevent moving up if at top limit
    return !topLimitSwitch.get();
  }

  public boolean isAtBottomLimit() {
    // Prevent moving down if at bottom limit
    // Note: limit switches typically read false when pressed, true when not pressed
    return !bottomLimitSwitch.get();
  }

  public void setAngle(double angleDegrees) {
    angleDegrees = Math.max(IntakeConstants.STOWED_POS,
                           Math.min(IntakeConstants.EXTENDED_POS, angleDegrees));

    double motorRotations = (angleDegrees / 360.0) / IntakeConstants.GEAR_RATIO;

    pivotMotor.setControl(positionControl.withPosition(motorRotations));
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
    // Command duty cycle
    pivotMotor.setControl(dutyCycleControl.withOutput(speed));
  }

   // Set motor output to zero & stop all movement
  public void stop() {
    pivotMotor.setControl(dutyCycleControl.withOutput(0));
  }

  //  Resets the encoder position to zero.
  public void zeroEncoder() {
    canCoder.setPosition(0.0);
  }

  // Set encoder position to the max angle
  public void setEncoderToMax() {
    double maxRotations = IntakeConstants.EXTENDED_POS / 360.0;
    canCoder.setPosition(maxRotations);
  }

  @Override
  public void periodic() {
    // Gets current state of top & bottom limit switches
    boolean topLimit = isAtTopLimit();
    boolean bottomLimit = isAtBottomLimit();

    if (bottomLimit && !previousBottomLimitState) {
      // Zero the encoder if bottom limit switch is just pressed
      zeroEncoder();
    }
    previousBottomLimitState = bottomLimit;

    if (topLimit && !previousTopLimitState) {
      // Set encoder to max if top limit switch is just pressed
      setEncoderToMax();
    }
    previousTopLimitState = topLimit;

    // SmartDashboard!
    
    // Position
    SmartDashboard.putNumber("Intake/Pivot/AngleDegrees", getAngleDegrees());
    SmartDashboard.putNumber("Intake/Pivot/AbsolutePosition", getAbsolutePosition());

    // Limit switches 
    SmartDashboard.putBoolean("Intake/Pivot/AtTopLimit", topLimit);
    SmartDashboard.putBoolean("Intake/Pivot/AtBottomLimit", bottomLimit);

    // Motor status
    SmartDashboard.putNumber("Intake/Pivot/DutyCycle", pivotMotor.get());
    SmartDashboard.putNumber("Intake/Pivot/Velocity", pivotMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Current", pivotMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Pivot/MotorTemp", pivotMotor.getDeviceTemp().getValueAsDouble());
  }
}