package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class table {

    private static InterpolatingDoubleTreeMap shooterAngle = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap shooterRPS = new InterpolatingDoubleTreeMap();

    public table() {
        shooterAngle.put(1.524, -0.3);
        shooterAngle.put(1.651, -0.3);
        shooterAngle.put(1.981, -0.3);
        shooterAngle.put(2.261, -0.3);
        shooterAngle.put(2.616, -0.27);

        shooterRPS.put(1.524,45.0);
        shooterRPS.put(1.651,45.0);
        shooterRPS.put(1.981,50.0);
        shooterRPS.put(2.261,50.0);
        shooterRPS.put(2.616, 60.0);

        
    }

    public static double getRPS(double dis){
        return shooterRPS.get(dis);
    }

    public static double getAngle(double dis){
        return shooterAngle.get(dis);
    }
}