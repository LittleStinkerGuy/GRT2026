package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class shooterLearner {

    private final NetworkTableEntry offsetEntry;
    private double offset = 0;

    public shooterLearner(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterLearner");
        offsetEntry = table.getEntry("offset");
        offsetEntry.setDouble(offset);
    }

    public double getRPM(double baseRPM){
        return baseRPM + offset;
    }

    public void updateUP(){
        offset += 5;
        offsetEntry.setDouble(offset);
    }

    public void updateDOWN(){
        offset -= 5;
        offsetEntry.setDouble(offset);
    }

    public void reset(){
        offset = 0;
        offsetEntry.setDouble(offset);
    }
    
}