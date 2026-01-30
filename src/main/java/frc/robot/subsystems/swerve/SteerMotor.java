package frc.robot.subsystems.swerve;

//Constants Import 
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import static frc.robot.Constants.SwerveSteerConstants.STEER_GEAR_REDUCTION;
import static frc.robot.Constants.SwerveSteerConstants.STEER_PEAK_CURRENT;
import static frc.robot.Constants.SwerveSteerConstants.STEER_RAMP_RATE;

import java.util.EnumSet;

import frc.robot.util.GRTUtil;

//Logging and NT imports


public class SteerMotor {

    // Motor instance for controlling the drive motor
    private TalonFX motor;

    // Configuration for Kraken stored in one Object
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration simplePIDConfig = new TalonFXConfiguration();

    // Cancoder Creation
    private CANcoder cancoder;

    // Configuration for CANCODER stored in one Object
    private final  CANcoderConfiguration cancoderconfig = new CANcoderConfiguration();

    // For fine control of velocity and torque using FOC (Field-Oriented Control)
    private PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0)
                .withSlot(0)
                .withFeedForward(0)
                .withUpdateFreqHz(100.0);
    // For making positions wrap from 0-1 and resetting to not stack
    private final ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();

    // For Logging
    private DoubleLogEntry temperatureLogEntry;
    private DoubleLogEntry motorPositionLogEntry;
    private DoubleLogEntry targetPositionLogEntry;
    private DoubleLogEntry appliedVoltsLogEntry;
    private DoubleLogEntry supplyCurrLogEntry;
    private DoubleLogEntry torqueCurrLogEntry;
    private DoubleLogEntry positionErrorLogEntry;

    // For NT
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

    private double gurtMotorPos = 0.0;
    private int gurtMotorCanID;
    // Phoenix 6 Status Signals
    private StatusSignal<Angle> positionSignal;
    private StatusSignal<Voltage> appliedVoltsSignal;
    private StatusSignal<Current> supplyCurrentSignal;
    private StatusSignal<Current> torqueCurrentSignal;


    public SteerMotor(int motorCAN, int encoderCAN) {
        // Set motor and encoder
        motor = new TalonFX(motorCAN, "can");
        cancoder = new CANcoder(encoderCAN, "can");
        gurtMotorCanID = motorCAN;
        // Configure CANcoder and Kraken
        configureCancoder(); // called to ensure settings are applied programmatically
        configureMotor();

        // Initialize NetworkTables
        initNT(motorCAN);


        // Initialize logs
        // initLogs(motorCAN);
        
        // Initialize Phoenix 6 signals
        initSignals();

    }

    /**
     * Applying Configurations to CANCODER
     */
    private void configureCancoder() {
        
        // cancoderconfig.MagnetSensor.MagnetOffset = offsetRads / (2.0 * Math.PI);
        cancoderconfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderconfig);

        // Apply config with retries (max 5 attempts)
        for (int i = 0; i < 5; i++) {
            if (cancoder.getConfigurator().apply(cancoderconfig, 0.1) == StatusCode.OK) {
                break; // Success
            }
        }
    }

    /**
     * Applying Configurations to Kraken
     */
    private void configureMotor() {

        // Set peak current for torque limiting for stall prevention
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = STEER_PEAK_CURRENT;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = - STEER_PEAK_CURRENT;

        // How fast can the code change torque for the motor
        motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = STEER_RAMP_RATE;

        // By Default Robot will not move
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // required if motor spins opposite 
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Encoder Being Applied

// GUYS WE DID NOT APPLY IT T_T

        // Use the CANcoder as feedback
        motorConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource   = FeedbackSensorSourceValue.RemoteCANcoder;

        // Tell it how rotor relates to module angle:
        motorConfig.Feedback.RotorToSensorRatio = STEER_GEAR_REDUCTION; // e.g., 12.8

        // Enable position wrapping (by default values are from 0-1)
        closedLoopGeneralConfigs.ContinuousWrap = true; //basicaly turns stacking off
        motorConfig.ClosedLoopGeneral = closedLoopGeneralConfigs;
        
        simplePIDConfig.Slot0.kP = 3;
        simplePIDConfig.Slot0.kI = 0.1;
        simplePIDConfig.Slot0.kD = 0;
        if (gurtMotorCanID == 1){
            simplePIDConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
            simplePIDConfig.Feedback.FeedbackSensorSource   = FeedbackSensorSourceValue.RemoteCANcoder;

            simplePIDConfig.Feedback.SensorToMechanismRatio = 1.0;
            simplePIDConfig.Feedback.RotorToSensorRatio = STEER_GEAR_REDUCTION;
            // simplePIDConfig.ClosedLoopGeneral.ContinuousWrap = true;

        }

        // Apply motor config with retries (max 5 attempts)
        for (int i = 0; i < 5; i++) {
            if (motor.getConfigurator().apply(simplePIDConfig, 0.1) == StatusCode.OK) {
                break; // Success
            }
            if (i == 4){
                System.out.println("VERY BAD MOTOR DID NOT GET CONFIGURED");
            }
        }
        
        // Reset motor position to 0 for consistent starting point
        motor.setPosition(0);
    }

    /**
     * Initializes Phoenix 6's signals
     */
    private void initSignals(){
        positionSignal = motor.getPosition();
        appliedVoltsSignal = motor.getMotorVoltage();
        torqueCurrentSignal = motor.getTorqueCurrent();
        supplyCurrentSignal = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            250.0, positionSignal, appliedVoltsSignal,
            torqueCurrentSignal, supplyCurrentSignal
        );
        motor.optimizeBusUtilization(0, 1.0);
    }

    private void initLogs(int canId) {
        temperatureLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "motorTemperature");
        motorPositionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "motorPosition");
        targetPositionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "targetPosition");
        appliedVoltsLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "appliedVolts");
        supplyCurrLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "supplyCurrent");
        torqueCurrLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "torqueCurrent");
        positionErrorLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "positionError");
    }

    public void logStats() {
        temperatureLogEntry.append(getTemperature(), GRTUtil.getFPGATime());
        motorPositionLogEntry.append(getPosition(), GRTUtil.getFPGATime());
        targetPositionLogEntry.append(getPosition(), GRTUtil.getFPGATime()); // TODO: Log actual target
        appliedVoltsLogEntry.append(appliedVoltsSignal.getValueAsDouble(), GRTUtil.getFPGATime());
        supplyCurrLogEntry.append(supplyCurrentSignal.getValueAsDouble(), GRTUtil.getFPGATime());
        torqueCurrLogEntry.append(torqueCurrentSignal.getValueAsDouble(), GRTUtil.getFPGATime());
        positionErrorLogEntry.append(0.0, GRTUtil.getFPGATime()); // TODO: Calculate actual position error
    }

    private void initNT(int canId) {
        ntInstance = NetworkTableInstance.getDefault();
        steerStatsTable = ntInstance.getTable("SwerveSteer");

        encoderPositionPublisher = steerStatsTable.getDoubleTopic(canId + "encoderPosition").publish();
        motorPositionPublisher = steerStatsTable.getDoubleTopic(canId + "motorPosition").publish();
        targetPositionPublisher = steerStatsTable.getDoubleTopic(canId + "targetPosition").publish();
        motorTemperaturePublisher = steerStatsTable.getDoubleTopic(canId + "motorTemperature").publish();
        appliedVoltsPublisher = steerStatsTable.getDoubleTopic(canId + "appliedVolts").publish();
        supplyCurrentPublisher = steerStatsTable.getDoubleTopic(canId + "supplyCurrent").publish();
        torqueCurrentPublisher = steerStatsTable.getDoubleTopic(canId + "torqueCurrent").publish();
        positionErrorPublisher = steerStatsTable.getDoubleTopic(canId + "positionError").publish();
        rotationPublisher = steerStatsTable.getDoubleTopic(canId + "targetMotorRotationPosition").publish();
        closedLoopReferencePublisher = steerStatsTable.getDoubleTopic(canId + "targetMotorRotationPosition").publish();
        gurtMotorPos1 = steerStatsTable.getDoubleTopic(canId + "motorPosThing").publish();
        gurtMotorPos1.set(0.0);
        positionControlPositionPublisher = steerStatsTable.getDoubleTopic(canId + "positionControlPosition").publish();
        steerStatsTable.addListener( canId + "motorPosThing", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) ->{
            gurtMotorPos = event.valueData.value.getDouble();
        });

    }

    public void publishStats() {
        motorPositionPublisher.set(getPosition());
  
        encoderPositionPublisher.set(cancoder.getPosition().getValueAsDouble());
        targetPositionPublisher.set(rotorRotations); // Just show current position for now
        rotationPublisher.set(motor.getPosition().getValueAsDouble()); // get position
        closedLoopReferencePublisher.set(motor.getClosedLoopReference().getValueAsDouble()); // TODO: Calculate actual position error
        
        motorTemperaturePublisher.set(getTemperature());
        appliedVoltsPublisher.set(appliedVoltsSignal.getValueAsDouble());
        supplyCurrentPublisher.set(supplyCurrentSignal.getValueAsDouble());
        torqueCurrentPublisher.set(torqueCurrentSignal.getValueAsDouble());
        positionErrorPublisher.set(0.0); // TODO: Calculate actual position error

        positionControlPositionPublisher.set(positionRequest.Position);
    
    }
    



    /**
     * Gets the motor's position through the absolute encoder
     * 
     * @return position in double from 0 to 1
     */
    public double getPosition() {
        return cancoder.getAbsolutePosition().getValueAsDouble();// 0..1 rotations, absolute
    }

    
    /**
     * Configures drive motor's PIDSV
     * @param p :Determines how much the config will react to the error
     * @param i :Corrects recurring errors over time by stacking past errors
     * @param d :Reacting to the rate of change of the error, preventing overshooting and damping oscillations
     * @param ff :
     */
    public void configPID(double p, double i, double d, double ff) {

        Slot0Configs slot0Configs = new Slot0Configs(); //used to store and update PID values

        /*
         * Think of P as how much we want it to correct, as an example imagine you are parking a car
         *      If you’re too far from the spot, you might turn the steering wheel sharply (a large correction) to get closer.
         *      If you’re very close to the spot, you might make smaller, finer adjustments (a smaller correction).
         * 
         * The proportional control is like how hard you turn the steering wheel based on how far you are from the target spot. If you’re far away, you turn more; if you’re close, you turn less.
         */

        slot0Configs.kP = p;

        /*
         * Integral Control's job is to correct recurring errors over time by stacking past errors.
         * It sums up previous errors, so it looks at how many errors you have had over time.
         * 
         * Imagine the motor is consistently below the target position due to friction.
         *      kI will accumulate this error over time and then apply this correction to bring the motor to its target.
         * 
         * Keep in mind that this will also apply if something is blocking the mechanism from working, e.g., a rock.
         * This will lead to a crazy increase in errors, potentially causing motors to burn out.
         */
        slot0Configs.kI = i;

        /*
         * The Derivitive Controls job is to look at the Rate of Change (slope) of how fast the error is changing (def of derrivitive)
         * If the error is chaning too fast, the kD will slow it down so we do not overshoot
         * 
         * Imagine you’re driving a car toward a stop sign. As you approach the stop sign, you start to apply the brakes. 
         *      The derivative control is like reacting to how quickly you’re approaching the stop sign:
         * 	        If you’re coming in fast, the derivative term would apply more braking force to slow you down before you overshoot the stop sign.
         *          If you’re coming in slowly, the derivative term would apply less braking force, just enough to prevent overshooting but not too much.
         * 
         * kD responds to the rate of change of the error, preventing overshooting and damping oscillations, too high will make mech slower
         */
        slot0Configs.kD = d;

        /*
        * Feedforward Control (kFF, or kV in Phoenix 6) predicts how much power we need based only on how fast we want to go,
        *      instead of waiting for an error to happen first.
        * 
        * Imagine you’re driving a car on the highway. 
        *      To stay at 60 mph, you don’t wait until you slow down to press the gas pedal — you give a little gas constantly.
        * 
        * kV is like setting a cruise control: it applies just enough output to maintain a certain speed or motion,
        *      before any errors even occur.
        * 
        * Feedforward makes the motor control smoother, faster, and prevents the controller from working too hard correcting errors later.
        */
        slot0Configs.kV = ff;
        
        motor.getConfigurator().apply(slot0Configs);
    }


    /**
     * Gets the tempature of the motor
     * @return temperature of the motor in double
     */
    public double getTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }
    

    private double degreesToMotorRotations(double degrees) {
        return (degrees / 360.0);
        // return (degrees / 360.0) / STEER_GEAR_REDUCTION;
    }

    /**
    * Gets the current position of the motor in radians.
    * 
    * @return the current position in radians
    */
    private double getCurrentPositionRads() {
        
        return getPosition() * 2.0 * Math.PI; // Convert from 0..1 rotations to radians
    }

    
    public double fasterTurnDirection(double current, double target) {  
        current = current / (Math.PI * 2)*360;
        target = target / (Math.PI * 2)*360;

        // Normalize the angles between 0 and 360
        current = ((current % 360) + 360) % 360;
        target = ((target % 360) + 360) % 360;

        // Calculate clockwise and counterclockwise distances
        double clockwise = (target - current + 360) % 360;
        double counterClockwise = (current - target + 360) % 360;

        // Determine the faster direction
        if (clockwise <= counterClockwise) {
            return clockwise*Math.PI*2/360; // turn clockwise
        } else {
            return -counterClockwise*Math.PI*2/360; // turn counterclockwise
        }
    }

    /**
     * Using PID to move to target position
     * 
     * @param targetRads target position in radiants
     */
    double rotorRotations;
    public void setPosition(double targetRads) {

        // double currentRads = getCurrentPositionRads(); 
        // double deltaRads = fasterTurnDirection(currentRads, targetRads);
        // double newTargetRads = currentRads + deltaRads;
        rotorRotations = (targetRads / (2.0 * Math.PI)*gurtMotorPos);
        
        

        positionRequest.withPosition(rotorRotations);
        motor.setControl(positionRequest);
        publishStats();
    }
}


