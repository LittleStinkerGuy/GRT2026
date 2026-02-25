package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.PolynomialRegression;

public final class VisionConstants {

    public static final double FIELD_X = 16.54;
    public static final double FIELD_Y = 8.07;
    public static final double ROBOT_RADIUS = 0.762;
    public static final double[] STD_DEV_DIST = new double[] {
      0.75, 1.00, 1.3, 1.69, 2., 2.51, 2.78, 3.07, 3.54, 4.1, 4.52
    };
    public static final double[] X_STD_DEV = new double[] {
      0.002, 0.005, 0.007, 0.014, 0.029, 0.074, 0.101, 0.12, 0.151, 0.204, 0.287
    };
    public static final double[] Y_STD_DEV = new double[] {
      0.002, 0.005, 0.013, 0.020, 0.067, 0.080, 0.095, 0.160, 0.206, 0.259, 0.288
    };
    public static final double[] O_STD_DEV = new double[] {
      0.002, 0.004, 0.005, 0.011, 0.031, 0.4, 1.72, 1.89, 2.05, 2.443, 2.804
    };

    public static final CameraConfig cameraConfig7 =
      new CameraConfig(
        "7",
        new Transform3d(
          0.28, 0, 0,
          new Rotation3d(0, -Math.toRadians(5), 0)
        )
      );
    public static final CameraConfig cameraConfig12 =
      new CameraConfig(
        "12",
        new Transform3d(
          0.28, 0, 0,
          new Rotation3d(0, -Math.toRadians(5), 0)
        )
      );
    public static final CameraConfig cameraConfig11 =
      new CameraConfig(
        "11",
        new Transform3d(
          -0.25, -0.313,0.339,
          new Rotation3d(new Quaternion(-0.6919944799721134, 0.1678468872622234, 0.15463356158232774, 0.6848792037701769))
        )
      );
    public static final CameraConfig cameraConfig6 =
      new CameraConfig(
        "6",
        new Transform3d(
          0.28, 0, 0,
          new Rotation3d(0, -Math.toRadians(5), 0)
        )
      );
    public static final PolynomialRegression xStdDevModel = new PolynomialRegression(
        STD_DEV_DIST,X_STD_DEV,2);
    public static final PolynomialRegression yStdDevModel = new PolynomialRegression(
        STD_DEV_DIST,Y_STD_DEV,2);
    public static final PolynomialRegression oStdDevModel = new PolynomialRegression(
        STD_DEV_DIST,O_STD_DEV,1);

  }
