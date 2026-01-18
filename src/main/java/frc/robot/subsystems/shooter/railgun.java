package frc.robot.subsystems.shooter;
import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.EnumSet;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class railgun extends SubsystemBase {

    
    private double velocity = 0;
    private double lastVelocity = 0;
   // private TalonFX lowerMotor = new TalonFX(railgunConstants.lowerWheelId, "can");
    private TalonFX upperMotor = new TalonFX(railgunConstants.upperId, "can");
    boolean readyToFire = false;
    boolean rPrevPress = false;
    boolean lPrevPress = false;
    private double potentialVel;
    private Pose2d ready;
    private Rotation2d wanted;
    private double distance = 0;
    private double height = 0;
    private VelocityVoltage spinner = new VelocityVoltage(0);;
    
    
    public railgun(){
        configNT();
    }

    public void configPID(double p, double i, double d, double ff) {

        Slot0Configs slot0Configs = new Slot0Configs(); //used to store and update PID values
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kV = ff;
        
        upperMotor.getConfigurator().apply(slot0Configs);
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

    private double calculateVel(){
        return Math.sqrt((railgunConstants.g*distance*distance)/(2*railgunConstants.cos75*railgunConstants.cos75*(distance*railgunConstants.tan75-(railgunConstants.height-height))));
    }

    public int alignAndCalculate(){

        //ready = po.position();
        //wanted = po.bearing();
        return 8;
    }

    public void input(boolean r, boolean l){ // check logic here again

        if(r && !rPrevPress){
            potentialVel = calculateVel();
            rPrevPress = true;
        }

        if(l && !lPrevPress){
            velocity = potentialVel;
            lPrevPress = true;
        }else if(!l){
            lPrevPress = false;
            velocity = 0;
        }

        if(!r){
            rPrevPress = false;
        }

    }

    public void periodic(){
         SmartDashboard.putNumber("Current Velocity", velocity);
         SmartDashboard.putBoolean("Ready To Fire?", readyToFire);
         SmartDashboard.putNumber("X Distance", distance);
         SmartDashboard.putNumber("Height of Launcher", height);
         distance = SmartDashboard.getNumber("X Distance", distance);
         height = SmartDashboard.getNumber("Height of Launcher", height);

         //duty cycle used here instead of PID bcs no target per se, want less lag, 
         if(velocity != lastVelocity){
           // lowerMotor.set(velocity);
           spinner.Velocity = velocity*railgunConstants.gearRatio/(2*Math.PI*railgunConstants.radius);
            upperMotor.setControl(spinner);
            lastVelocity = velocity;
         }
    }

} 