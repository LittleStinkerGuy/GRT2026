package frc.robot.subsystems.shooter;
import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.EnumSet;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class railgun extends SubsystemBase {

    
    private double velocity = 0;
    private TalonFX lowerMotor = new TalonFX(railgunConstants.lowerId, "can");
    private TalonFX upperMotor = new TalonFX(railgunConstants.upperId, "can");
    private TalonFX hoodMotor = new TalonFX(railgunConstants.hoodId, "can");
    boolean readyToFire = false;
    boolean rPrevPress = false;
    private double potentialVel;
    private Pose2d ready;
    private Rotation2d wanted;
    private double distance = 0;
    private double height = 0;
    private VelocityVoltage spinner = new VelocityVoltage(0);
    private VelocityVoltage low = new VelocityVoltage(0);
    private PositionTorqueCurrentFOC focThing = new PositionTorqueCurrentFOC(0);
    int motorTuning = 1;
    boolean manual = false;
    double hoodAngle = 75;
    
    
    public railgun(){
        configNT();
        configure();
    }

    public void configPID(double p, double i, double d, double ff) {

        Slot0Configs slot0Configs = new Slot0Configs(); //used to store and update PID values
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kV = ff;
        
        if(motorTuning == 1){lowerMotor.getConfigurator().apply(slot0Configs);}
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
    }

    private double calculateVel(){
        return Math.sqrt((railgunConstants.g*distance*distance)/(2*railgunConstants.cos75*railgunConstants.cos75*(distance*railgunConstants.tan75-(railgunConstants.height-height))));
    }

    public int alignAndCalculate(){

        //ready = po.position();
        //wanted = po.bearing();
        return 8;
    }

    boolean prevOptions = false;
    public void input(double r, boolean l, int arrow, boolean options){ // check logic here again

        if(options && !prevOptions){
            manual = !manual;
            prevOptions = true;
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
            if(spinner.Velocity >= 0){low.Velocity = railgunConstants.velocityLow;};

        }else{

            spinner.Velocity = velocity*railgunConstants.gearRatioUpper/(2*Math.PI*railgunConstants.radius);
        }

    }

    public void periodic(){
         SmartDashboard.putNumber("Current Velocity", velocity);
         SmartDashboard.putBoolean("Ready To Fire?", readyToFire);
         SmartDashboard.putNumber("X Distance", distance);
         SmartDashboard.putNumber("Height of Launcher", height);
         distance = SmartDashboard.getNumber("X Distance", distance);         // these guys
         height = SmartDashboard.getNumber("Height of Launcher", height);     // just for testing
         SmartDashboard.putNumber("Which Motor Tuning", motorTuning);
         motorTuning = (int) SmartDashboard.getNumber("Which Motor Tuning", motorTuning);


         //just reset every 20 ms, simpler that way, and apparently this is how it was meant to be done
         lowerMotor.setControl(new VelocityVoltage(low.Velocity));
         upperMotor.setControl(new VelocityVoltage(spinner.Velocity));
         hoodMotor.setControl(focThing.withPosition(hoodAngle));
        
    }

} 