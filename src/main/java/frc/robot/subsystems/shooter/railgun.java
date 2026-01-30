package frc.robot.subsystems.shooter;

import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;

public class railgun extends SubsystemBase {

    private final TalonFX upperMotor;
    private final TalonFX hoodMotor;

    public railgun(CANBus cn) {
        // Construct motors directly on the CAN bus
        upperMotor = new TalonFX(railgunConstants.upperId, cn);
        hoodMotor = new TalonFX(railgunConstants.hoodId, cn);

        // Initialize hood to starting angle
        hoodMotor.setPosition(railgunConstants.initHoodAngle);
    }

    /**
     * Manual input function.
     * @param r R2 axis for upper motor (-1 to 1)
     * @param arrow POV for hood control (0 = up, 180 = down)
     */
    public void input(double r, int arrow) {
        // Hood control
        if (arrow == 0) {
            hoodMotor.set(-0.2); // move up
        } else if (arrow == 180) {
            hoodMotor.set(0.2);  // move down
        } else {
            hoodMotor.set(0);    // stop
        }

        upperMotor.set((r + 1) / 2);

        // Optional debug
        SmartDashboard.putNumber("Hood Position", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Upper Motor Output", (r + 1) / 2);
    }

    @Override
    public void periodic() {
        // Nothing else needed for manual mode
    }
}
