package frc.robot.subsystems.Vision;


import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
    private String cameraName;
    private Transform3d cameraPose;
    
    public CameraConfig(
        String cameraName, Transform3d cameraPose
    ){
        this.cameraName = cameraName;
        this.cameraPose = cameraPose;
    }

    public String getCameraName(){
        return cameraName;
    }

    public Transform3d getCameraPose(){
        return cameraPose;
    }

}
