package frc.robot.subsystems.swerve;

import java.util.Queue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.GatedAlert;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PhoenixUtil;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    private static final String PIGEON_ALERT_PREFIX = "Swerve Pigeon (ID " + SwerveConstants.PIGEON_ID + "): ";

    private boolean pigeonConnected = false;

    private final Alert pigeonDisconnectedAlert = new Alert(PIGEON_ALERT_PREFIX + "Disconnected", AlertType.kError);
    private final GatedAlert failedToConfigureAlert = new GatedAlert(PIGEON_ALERT_PREFIX + "Failed to configure", AlertType.kError, () -> pigeonConnected);

    public GyroIOPigeon2(LoggedCanivore canivore) {
        pigeon = new Pigeon2(SwerveConstants.PIGEON_ID, canivore);
        PhoenixUtil.tryUntilOk(5, () -> pigeon.getConfigurator().apply(new Pigeon2Configuration()), failedToConfigureAlert);
        pigeon.getConfigurator().setYaw(0.0);

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        // FD allows bus to go at 250 Hz
        double yawUpdateFrequency = canivore.isNetworkFD() ? 250 : 100;
        yaw.setUpdateFrequency(yawUpdateFrequency);

        yawVelocity.setUpdateFrequency(120.0);
        pigeon.optimizeBusUtilization();

        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());

        refreshPigeonAlerts(BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPositionDeg = yaw.getValueAsDouble();
        inputs.yawVelocityDegPerSec = yawVelocity.getValueAsDouble();

        refreshPigeonAlerts(inputs.connected);

        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
            yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    private void refreshPigeonAlerts(boolean connected) {
        pigeonConnected = connected;
        pigeonDisconnectedAlert.set(!connected);
        failedToConfigureAlert.push();
    }
}
