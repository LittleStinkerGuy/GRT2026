package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveSteerConstants.STEER_ACCELERATION;
import static frc.robot.Constants.SwerveSteerConstants.STEER_CRUISE_VELOCITY;
import static frc.robot.Constants.SwerveSteerConstants.STEER_CURRENT_LIMIT_ENABLE;
import static frc.robot.Constants.SwerveSteerConstants.STEER_GEAR_REDUCTION;
import static frc.robot.Constants.SwerveSteerConstants.STEER_PEAK_CURRENT;
import static frc.robot.Constants.SwerveSteerConstants.STEER_RAMP_RATE;
import static frc.robot.Constants.SwerveSteerConstants.STEER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.SwerveSteerConstants.STEER_SUPPLY_CURRENT_LIMIT;

import java.util.EnumSet;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GRTUtil;

public class SteerMotor2 extends SubsystemBase{
        // For NT

    //test two 
    private NetworkTableInstance ntInstance;
    private NetworkTable steerStatsTable;
    private DoublePublisher encoderPositionPublisher;
    private DoublePublisher motorPositionPublisher;
    private DoublePublisher targetPositionPublisher;
    private DoublePublisher motorTemperaturePublisher;
    private DoublePublisher appliedVoltsPublisher;
    private DoublePublisher supplyCurrentPublisher;
    private DoublePublisher torqueCurrentPublisher;
    private DoublePublisher positionErrorPublisher;
    private DoublePublisher rotationPublisher;
    private DoublePublisher positionControlPositionPublisher;
    private DoublePublisher closedLoopReferencePublisher;
    private DoublePublisher gurtMotorPos1;
    private NetworkTableEntry motorNewPos;
    
    // DataLog entries
    private DoubleLogEntry positionLogEntry;
    private DoubleLogEntry velocityLogEntry;
    private DoubleLogEntry targetPositionLogEntry;
    private DoubleLogEntry appliedVoltsLogEntry;
    private DoubleLogEntry supplyCurrentLogEntry;
    private DoubleLogEntry torqueCurrentLogEntry;
    private DoubleLogEntry temperatureLogEntry;
    private DoubleLogEntry closedLoopErrorLogEntry;

    private double gurtMotorPos = 0.0;
    private double targetPos = 0.0;
    private int canID;
    private int motorID;
    private TalonFX motor;
    private CANcoder cancoder;
    private final boolean enableEncoder = true;
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration encoderConfig  = new CANcoderConfiguration();
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0)
                .withSlot(0)
                .withUpdateFreqHz(100.0);

    private void configureMotor() {
        // Set peak current for torque limiting for stall prevention
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = STEER_PEAK_CURRENT;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -STEER_PEAK_CURRENT;

        // Current limits (optimized for swerve steer)
        motorConfig.CurrentLimits.SupplyCurrentLimit = STEER_SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = STEER_CURRENT_LIMIT_ENABLE;
        motorConfig.CurrentLimits.StatorCurrentLimit = STEER_STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = STEER_CURRENT_LIMIT_ENABLE;

        // How fast can the code change torque for the motor
        motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = STEER_RAMP_RATE;

        // By Default Robot will not move
        motorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
 

        // motorConfig.Slot0.kP = 3;
        // motorConfig.Slot0.kI = 0;
        // motorConfig.Slot0.kD = 0;
        // Encoder Being Applied
        if (enableEncoder){

        // Use the CANcoder as feedback
        motorConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource   = FeedbackSensorSourceValue.RemoteCANcoder;
        }
        // Tell it how rotor relates to module angle:
        // motorConfig.Feedback.SensorToMechanismRatio = STEER_GEAR_REDUCTION; // e.g., 12.8
        
        // Enable position wrapping (by default values are from 0-1)

        // MotionMagic profile config
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = STEER_CRUISE_VELOCITY;
        motorConfig.MotionMagic.MotionMagicAcceleration = STEER_ACCELERATION;

        // Apply motor config with retries (max 5 attempts)
        for (int i = 0; i < 5; i++) {
            if (motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK) {
                System.out.println("MOTOR" + motorID + "SUCCESSFULLY CONFIGURED");

                break; // Success
            }
            if (i == 4){
                System.out.println("VERY BAD MOTOR" + motorID + "DID NOT GET CONFIGURED");
            }
        }
        
        // Reset motor position to 0 for consistent starting point
        motor.setPosition(0);
    }
    

    public void configPID(double p, double i, double d, double s) {

        Slot0Configs slot0Configs = new Slot0Configs(); //used to store and update PID values
        /*
         * Think of P as how much we want it to correct, as an example imagine you are parking a car
         */

        slot0Configs.kP = p;
        /*
         * Integral Control's job is to correct recurring errors over time by stacking past errors.
         * It sums up previous errors, so it looks at how many errors you have had over time.
         */
        slot0Configs.kI = i;

        /*
         * The Derivitive Controls job is to look at the Rate of Change (slope) of how fast the error is changing (def of derrivitive)
         * If the error is chaning too fast, the kD will slow it down so we do not overshoot
         */
        slot0Configs.kD = d;

        /*
        * Feedforward Control (kS, or kV in Phoenix 6) predicts how much power we need based only on how fast we want to go,
        *      instead of waiting for an error to happen first.
        */
        slot0Configs.kS = s;
        
        motor.getConfigurator().apply(slot0Configs);
    }
    private void initNT(int canId) {
        ntInstance = NetworkTableInstance.getDefault();
        steerStatsTable = ntInstance.getTable("SwerveSteer");
        motorNewPos = steerStatsTable.getEntry(canId + "motorPosThing");

        // encoderPositionPublisher = steerStatsTable.getDoubleTopic(canId + "encoderPosition").publish();
        motorPositionPublisher = steerStatsTable.getDoubleTopic(canId + "motorPosition").publish();
        targetPositionPublisher = steerStatsTable.getDoubleTopic(canId + "targetPosition").publish();
        // motorTemperaturePublisher = steerStatsTable.getDoubleTopic(canId + "motorTemperature").publish();
        // appliedVoltsPublisher = steerStatsTable.getDoubleTopic(canId + "appliedVolts").publish();
        // supplyCurrentPublisher = steerStatsTable.getDoubleTopic(canId + "supplyCurrent").publish();
        // torqueCurrentPublisher = steerStatsTable.getDoubleTopic(canId + "torqueCurrent").publish();
        // positionErrorPublisher = steerStatsTable.getDoubleTopic(canId + "positionError").publish();
        rotationPublisher = steerStatsTable.getDoubleTopic(canId + "controllerTargetPosition").publish();
        // closedLoopReferencePublisher = steerStatsTable.getDoubleTopic(canId + "targetMotorRotationPosition").publish();
        gurtMotorPos1 = steerStatsTable.getDoubleTopic(canId + "motorPosThing").publish();
        gurtMotorPos1.set(0.0);
        positionControlPositionPublisher = steerStatsTable.getDoubleTopic(canId + "positionControlPosition").publish();
        steerStatsTable.addListener( canId + "motorPosThing", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) ->{
            gurtMotorPos = event.valueData.value.getDouble();
        });

    }

    public void publishStats() {

        motorPositionPublisher.set(motor.getPosition().getValueAsDouble());
        targetPositionPublisher.set(targetPos);

        // encoderPositionPublisher.set(cancoder.getPosition().getValueAsDouble());
        // targetPositionPublisher.set(rotorRotations); // Just show current position for now
        rotationPublisher.set(controllerTargetRotations); // get position
        // closedLoopReferencePublisher.set(motor.getClosedLoopReference().getValueAsDouble()); // TODO: Calculate actual position error
        
        // positionErrorPublisher.set(0.0); // TODO: Calculate actual position error

        // positionControlPositionPublisher.set(positionRequest.Position);
    
    }

    public SteerMotor2(int motorCAN, int encoderID, CANBus canivore){
        motorID = motorCAN;
        motor = new TalonFX(motorCAN, canivore);
        cancoder = new CANcoder(encoderID);
        configureMotor();
        initNT(motorCAN);
        initLogs(motorCAN);
    }

    private void initLogs(int canId) {
        positionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "steer/" + canId + "/position");
        velocityLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "steer/" + canId + "/velocityRPM");
        targetPositionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "steer/" + canId + "/targetPosition");
        appliedVoltsLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "steer/" + canId + "/appliedVolts");
        supplyCurrentLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "steer/" + canId + "/supplyCurrent");
        torqueCurrentLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "steer/" + canId + "/torqueCurrent");
        temperatureLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "steer/" + canId + "/temperature");
        closedLoopErrorLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "steer/" + canId + "/closedLoopError");
    }

    public void logStats() {
        long ts = GRTUtil.getFPGATime();
        positionLogEntry.append(motor.getPosition().getValueAsDouble(), ts);
        velocityLogEntry.append(motor.getVelocity().getValueAsDouble() * STEER_GEAR_REDUCTION * 60.0, ts);
        targetPositionLogEntry.append(gurtMotorPos, ts);
        appliedVoltsLogEntry.append(motor.getMotorVoltage().getValueAsDouble(), ts);
        supplyCurrentLogEntry.append(motor.getSupplyCurrent().getValueAsDouble(), ts);
        torqueCurrentLogEntry.append(motor.getTorqueCurrent().getValueAsDouble(), ts);
        temperatureLogEntry.append(motor.getDeviceTemp().getValueAsDouble(), ts);
        closedLoopErrorLogEntry.append(motor.getClosedLoopError().getValueAsDouble(), ts);
    }
    /**
     * 
     * @param C current rotations domain: 0-1
     * @param T target rotations domain: 0-1
     * @return MotorPosition target range: -1 - 1
     */
    public double getOptimalSteerTargetPosition(double C, double T){

        double d1 = Math.abs((T+1)-C);//T+1 
        double d2 = Math.abs((T) - C);//T
        double d3 = Math.abs((T-1) - C);
        double motorPos = 0;
        if ((d1 <= d2)&&(d1 <= d3)){
            motorPos = T+1;
        } 
        if ((d2 <= d1)&&(d2<=d3)){
            motorPos = T;
        }
        if ((d3 <= d1)&&(d3<=d2)){
            motorPos = T-1;
        }
        return motorPos;
    }

    
    /**
     * 
     * @param targetWheelPosition wheel position in radians, pi = 180 degrees CCW looking from the top
     */
    double controllerTargetRotations;
    public void setPosition(double targetWheelPosition){
        gurtMotorPos = targetWheelPosition;

        targetWheelPosition = (targetWheelPosition / (2*Math.PI)) + .5;
        controllerTargetRotations = targetWheelPosition;
        // System.out.println("moved: " + gurtMotorPos);
        gurtMotorPos = targetWheelPosition;

        targetWheelPosition = targetWheelPosition % 1;

        //radians to rotations
        // // motor.wra(motorCurrentPos);

        // targetWheelPosition = getOptimalSteerTargetPosition(motorCurrentPos, targetWheelPosition);        
        // targetPos = targetWheelPosition;
        motionMagicRequest.withPosition(gurtMotorPos);
        motor.setControl(motionMagicRequest);
        publishStats();
    }
    /**
     * 
     * @return get position range 0-1
     */
    public double getPosition(){
        double motorCurrentPos = motor.getPosition().getValueAsDouble();
        //ensures current motor position is between 0 and 1
        return motorCurrentPos;
        }

    public double getVelocityRPM() {
        return motor.getVelocity().getValueAsDouble() * STEER_GEAR_REDUCTION * 60.0;
    }

    public void setCruiseVelocity(double velocity) {
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicCruiseVelocity = velocity;
        mmConfigs.MotionMagicAcceleration = STEER_ACCELERATION;
        motor.getConfigurator().apply(mmConfigs);
        System.out.println("MOTOR " + motorID + " cruise velocity: " + (velocity * STEER_GEAR_REDUCTION * 60.0) + " RPM");
    }

    public void setCruiseVelocity(double velocity, double acceleration) {
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicCruiseVelocity = velocity;
        mmConfigs.MotionMagicAcceleration = acceleration;
        motor.getConfigurator().apply(mmConfigs);
    }
}