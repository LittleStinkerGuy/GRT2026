package frc.robot.subsystems.shooter;
import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.EnumSet;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class railgun extends SubsystemBase {

    
    private double velocity = 0;
    private TalonFX upperMotor = new TalonFX(railgunConstants.upperId, "can");
    private TalonFX hoodMotor = new TalonFX(railgunConstants.hoodId, "can");
    private CANdi limit = new CANdi(railgunConstants.limitId, "can");
    private final CANcoder hoodEncoder = new CANcoder(railgunConstants.hoodEncoderId, "can");
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    TalonFXConfiguration upperConfig = new TalonFXConfiguration();
    private final CANcoder upperEncoder = new CANcoder(railgunConstants.upperEncoderId, "can");

    private VelocityVoltage spinner = new VelocityVoltage(0);
    private PositionTorqueCurrentFOC focThing = new PositionTorqueCurrentFOC(0);
    public SwerveSubsystem swerve;

    private double dist = 0;
    private double height = 0;
    double hoodAngle = 75;

    boolean manual = false;
    
    
    public railgun(SwerveSubsystem s){
        configNT();
        configure();
        swerve = s;
    }

    int motorTuning = 1;
    public void configPID(double p, double i, double d, double ff) {

        Slot0Configs slot0Configs = new Slot0Configs(); //used to store and update PID values
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kV = ff;
        
        if(motorTuning == 1){/*lowerMotor.getConfigurator().apply(slot0Configs);*/}
        else if(motorTuning == 2){upperMotor.getConfigurator().apply(slot0Configs);}
        else if(motorTuning == 3){hoodMotor.getConfigurator().apply(slot0Configs);}
    }


    private void configNT(){
        NetworkTableInstance.getDefault().getTable("intakeDEBUG")
                .getEntry("PIDF")
                .setDoubleArray(
                    new double[] {
                        45,
                        15,
                        0,
                        0.0
                    }
                );
        NetworkTableInstance.getDefault().getTable("intakeDEBUG").addListener(
                "PIDF",
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                (table, key, event) -> {
                    double[] pidf = event.valueData.value.getDoubleArray();
                    configPID(pidf[0], pidf[1], pidf[2], pidf[3]);
                }
            );
    }

    private void configure(){
        //set gear ratios
        //set init positions
        
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.MagnetOffset = railgunConstants.hoodMagnetOffset; 
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        hoodEncoder.getConfigurator().apply(cfg);
        hoodEncoder.setPosition(railgunConstants.initHoodAngle);

        FeedbackConfigs fb = new FeedbackConfigs();
        fb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        fb.FeedbackRemoteSensorID = railgunConstants.hoodEncoderId;
        fb.SensorToMechanismRatio = railgunConstants.gearRatioHood;
        hoodMotor.getConfigurator().apply(fb);

        FeedbackConfigs b = new FeedbackConfigs();
        b.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        b.FeedbackRemoteSensorID = railgunConstants.upperEncoderId;
        b.SensorToMechanismRatio = railgunConstants.gearRatioUpper;
        upperMotor.getConfigurator().apply(b);

        hoodConfig.Slot0.kP = 0.2;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;
        hoodConfig.Slot0.kG = 1.0;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CurrentLimitsConfigs currLim = new CurrentLimitsConfigs().withStatorCurrentLimit(50.0).withStatorCurrentLimitEnable(true);
        hoodConfig.withCurrentLimits(currLim);
        hoodConfig.Feedback.SensorToMechanismRatio = railgunConstants.gearRatioHood;
        hoodMotor.getConfigurator().apply(hoodConfig);

        upperConfig.Slot0.kP = 0.2;
        upperConfig.Slot0.kI = 0.0;
        upperConfig.Slot0.kD = 0.0;
        upperConfig.Slot0.kG = 1.0;
        upperConfig.Slot0.kV = 1.0;
        upperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        currLim = new CurrentLimitsConfigs().withStatorCurrentLimit(50.0).withStatorCurrentLimitEnable(true);
        upperConfig.withCurrentLimits(currLim);
        upperConfig.Feedback.SensorToMechanismRatio = railgunConstants.gearRatioHood;
        upperMotor.getConfigurator().apply(upperConfig);
    }

    private double calculateAngle(double dista){
        return Math.toDegrees(Math.atan( (6/(dista))*(1+ Math.sqrt(1- (1.8288/3)) ) ));
    }

    private double calculateVel(double ang){
        return Math.sqrt(6*9.8/ ((Math.toDegrees(Math.sin(Math.toRadians(ang))))*(Math.toDegrees(Math.sin(Math.toRadians(ang))))) );
    }


    boolean prevOptions = false;
    boolean prevL = false;
    public void input(double r, boolean l, int arrow, boolean options){ // check logic here again

        if(limit.getS1Closed().refresh().getValue()){
            hoodEncoder.setPosition(railgunConstants.initHoodAngle);
            hoodMotor.setPosition(railgunConstants.initHoodAngle);
        }

        if(options && !prevOptions){
            manual = !manual;
            prevOptions = true;
        }

        if(l && !prevL){
            prevL = true;
            swerve.facePose();
        }

        if(!l){
            prevL = false;
        }

        if(!options){prevOptions = false;}

        if(manual){

            //hood
            if(arrow == 0 && hoodAngle + 0.014 <= railgunConstants.upperAngle){
                hoodAngle += 0.014;
            }else if(arrow == 180 && hoodAngle - 0.014 >= railgunConstants.lowerAngle){
                hoodAngle -= 0.014;
            }

            //vel
            spinner.Velocity = railgunConstants.maxVelo * r;

        }else{

            dist = swerve.getRobotPosition().getTranslation().getDistance(railgunConstants.hubPos.getTranslation());

            hoodAngle = calculateAngle(dist)/360;
            velocity = calculateVel(hoodAngle*360);

            if(r > 0){
                spinner.Velocity = velocity*railgunConstants.gearRatioUpper/(2*Math.PI*railgunConstants.radius);
            }else{
                spinner.Velocity = 0;
            }

        }

    }

    public void periodic(){
        SmartDashboard.putNumber("Current Velocity", velocity);
        SmartDashboard.putNumber("X Distance", dist);
        dist = SmartDashboard.getNumber("X Distance", dist);         // these guys
        SmartDashboard.putBoolean("Mode", manual);
        SmartDashboard.putNumber("Which Motor Tuning", motorTuning);   // just for testing
        motorTuning = (int) SmartDashboard.getNumber("Which Motor Tuning", motorTuning);


         //just reset every 20 ms, simpler that way, and apparently this is how it was meant to be done
         //lowerMotor.setControl(new VelocityVoltage(low.Velocity));
        upperMotor.setControl(new VelocityVoltage(spinner.Velocity));
        hoodMotor.setControl(focThing.withPosition(hoodAngle)); 
        
    }

} 
