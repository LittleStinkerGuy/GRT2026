package frc.robot.subsystems.Vision;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BallDetectionSubsystem extends SubsystemBase {

    private static final int SMOOTHING_WINDOW_SIZE = 5;
    private static final double HOLD_TIME_SECONDS = 0.2;
    private static final double DECAY_TIME_SECONDS = 0.4;

    public record Detection(
        double timestampSeconds,
        double yawDegrees,
        double pitchDegrees,
        double area,
        double distanceMeters
    ) { }

    private final PhotonCamera camera;
    private int pipelineIndex;
    private final double cameraHeightMeters;
    private final double targetHeightMeters;
    private final double cameraPitchRadians;
    private final boolean rangeCalculationEnabled;

    private final String dashboardPrefix;

    private Consumer<Detection> detectionConsumer = (d) -> {};

    private List<Detection> latestDetections = List.of();
    private Optional<Detection> bestDetection = Optional.empty();

    private final Deque<Double> yawWindow = new ArrayDeque<>();
    private final Deque<Double> pitchWindow = new ArrayDeque<>();
    private double yawWindowSum = 0.0;
    private double pitchWindowSum = 0.0;
    private double filteredYaw = Double.NaN;
    private double filteredPitch = Double.NaN;
    private double filteredDistance = Double.NaN;
    private double filteredTimestamp = Double.NaN;
    private double lastDetectionRobotTime = Double.NaN;
    private double holdUntilRobotTime = Double.NEGATIVE_INFINITY;

    /**
     * Creates the subsystem using the default configuration (pipeline 0) and no
     * range calculation.
     *
     * @param cameraName PhotonVision camera name to use
     */
    public BallDetectionSubsystem(String cameraName) {
        this(BallDetectionConfig.defaultConfig(cameraName));
    }

    /**
     * Creates the subsystem with a custom configuration.
     * 
     * @param config user supplied configuration
     */
    public BallDetectionSubsystem(BallDetectionConfig config) {
        Objects.requireNonNull(config, "BallDetectionConfig cannot be null");

        camera = new PhotonCamera(config.cameraName());
        pipelineIndex = config.pipelineIndex();
        cameraHeightMeters = config.cameraHeightMeters();
        targetHeightMeters = config.targetHeightMeters();
        cameraPitchRadians = config.cameraPitchRadians();
        rangeCalculationEnabled = !(
            Double.isNaN(cameraHeightMeters) ||
            Double.isNaN(targetHeightMeters) ||
            Double.isNaN(cameraPitchRadians)
        );

        dashboardPrefix = "Ball Detection/" + config.cameraName() + "/";
        camera.setPipelineIndex(pipelineIndex);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            publishTelemetry();
            return;
        }

        for (PhotonPipelineResult result : results) {
            handleResult(result);
        }
    }

    /**
     * Assigns the consumer that should be notified whenever a new best detection is
     * produced. Passing {@code null} clears the consumer.
     */
    public void setBestDetectionConsumer(Consumer<Detection> consumer) {
        detectionConsumer = consumer != null ? consumer : (d) -> {};
    }

    /**
     * Updates which PhotonVision pipeline index should run.
     */
    public void setPipelineIndex(int newPipelineIndex) {
        if (newPipelineIndex == pipelineIndex) {
            return;
        }
        pipelineIndex = newPipelineIndex;
        camera.setPipelineIndex(pipelineIndex);
    }

    /**
     * Gets the active pipeline index used by the camera.
     */
    public int getPipelineIndex() {
        return pipelineIndex;
    }

    /**
     * Returns an immutable view of the detections from the most recent pipeline
     * result processed by the subsystem.
     */
    public List<Detection> getDetections() {
        return List.copyOf(latestDetections);
    }

    /**
     * Returns the best detection (if any) from the latest processed result.
     */
    public Optional<Detection> getBestDetection() {
        return bestDetection;
    }

    private void handleResult(PhotonPipelineResult result) {
        List<Detection> processed = new ArrayList<>();

        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                processed.add(createDetection(result.getTimestampSeconds(), target));
            }
        }

        latestDetections = List.copyOf(processed);
        double now = Timer.getFPGATimestamp();
        if (processed.isEmpty()) {
            bestDetection = Optional.empty();
            applyDecayToFilteredValues(now);
        } else {
            bestDetection = processed.stream()
                .max(Comparator.comparingDouble(Detection::area));
            if (bestDetection.isPresent()) {
                Detection detection = bestDetection.get();
                recordDetectionForSmoothing(detection, now);
                detectionConsumer.accept(detection);
            } else {
                applyDecayToFilteredValues(now);
            }
        }
        publishTelemetry();
    }

    private Detection createDetection(double timestampSeconds, PhotonTrackedTarget target) {
        double distanceMeters = Double.NaN;
        if (rangeCalculationEnabled) {
            distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeightMeters,
                targetHeightMeters,
                cameraPitchRadians,
                Units.degreesToRadians(target.getPitch())
            );
        }
        return new Detection(
            timestampSeconds,
            target.getYaw(),
            target.getPitch(),
            target.getArea(),
            distanceMeters
        );
    }

    private void recordDetectionForSmoothing(Detection detection, double robotTimestampSeconds) {
        yawWindowSum = appendSample(yawWindow, detection.yawDegrees(), yawWindowSum);
        pitchWindowSum = appendSample(pitchWindow, detection.pitchDegrees(), pitchWindowSum);

        filteredYaw = yawWindowSum / yawWindow.size();
        filteredPitch = pitchWindowSum / pitchWindow.size();

        double detectionDistance = detection.distanceMeters();
        if (!Double.isNaN(detectionDistance)) {
            if (Double.isNaN(filteredDistance)) {
                filteredDistance = detectionDistance;
            } else {
                filteredDistance = 0.7 * filteredDistance + 0.3 * detectionDistance;
            }
        }
        filteredTimestamp = detection.timestampSeconds();
        lastDetectionRobotTime = robotTimestampSeconds;
        holdUntilRobotTime = robotTimestampSeconds + HOLD_TIME_SECONDS;
    }

    private void applyDecayToFilteredValues(double nowSeconds) {
        if (!Double.isFinite(lastDetectionRobotTime)) {
            filteredYaw = Double.NaN;
            filteredPitch = Double.NaN;
            filteredDistance = Double.NaN;
            filteredTimestamp = Double.NaN;
            yawWindow.clear();
            pitchWindow.clear();
            yawWindowSum = 0.0;
            pitchWindowSum = 0.0;
            return;
        }

        if (nowSeconds <= holdUntilRobotTime) {
            return;
        }

        double elapsed = nowSeconds - holdUntilRobotTime;
        double decayProgress = Math.min(elapsed / DECAY_TIME_SECONDS, 1.0);
        double scale = Math.max(0.0, 1.0 - decayProgress);

        if (!Double.isNaN(filteredYaw)) {
            filteredYaw *= scale;
        }
        if (!Double.isNaN(filteredPitch)) {
            filteredPitch *= scale;
        }
        if (!Double.isNaN(filteredDistance)) {
            filteredDistance *= scale;
        }

        if (decayProgress >= 1.0) {
            filteredYaw = Double.NaN;
            filteredPitch = Double.NaN;
            filteredDistance = Double.NaN;
            filteredTimestamp = Double.NaN;
            yawWindow.clear();
            pitchWindow.clear();
            yawWindowSum = 0.0;
            pitchWindowSum = 0.0;
            lastDetectionRobotTime = Double.NaN;
            holdUntilRobotTime = Double.NEGATIVE_INFINITY;
        }
    }

    private double appendSample(Deque<Double> window, double value, double currentSum) {
        window.addLast(value);
        currentSum += value;
        if (window.size() > SMOOTHING_WINDOW_SIZE) {
            currentSum -= window.removeFirst();
        }
        return currentSum;
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber(key("count"), latestDetections.size());

        double[] yawSamples = latestDetections.stream()
            .mapToDouble(Detection::yawDegrees)
            .toArray();
        double[] pitchSamples = latestDetections.stream()
            .mapToDouble(Detection::pitchDegrees)
            .toArray();
        SmartDashboard.putNumberArray(key("yawSamples"), yawSamples);
        SmartDashboard.putNumberArray(key("pitchSamples"), pitchSamples);

        SmartDashboard.putNumber(key("bestYawDeg"), filteredYaw);
        SmartDashboard.putNumber(key("bestPitchDeg"), filteredPitch);
        SmartDashboard.putNumber(key("bestDistanceMeters"), filteredDistance);
        SmartDashboard.putNumber(key("timestampSeconds"), filteredTimestamp);
    }

    private String key(String suffix) {
        return dashboardPrefix + suffix;
    }

    /**
     * Configuration record used to describe the colored-shape pipeline setup.
     * Values may be {@link Double#NaN} when the corresponding configuration is not
     * available (for example when range calculation is not desired).
     */
    public static record BallDetectionConfig(
        String cameraName,
        double cameraHeightMeters,
        double targetHeightMeters,
        double cameraPitchRadians,
        int pipelineIndex
    ) {
        public BallDetectionConfig {
            Objects.requireNonNull(cameraName, "cameraName is required");
        }

        /**
         * Creates a default configuration for the supplied camera.
         */
        public static BallDetectionConfig defaultConfig(String cameraName) {
            return new BallDetectionConfig(
                cameraName,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                0
            );
        }

        public BallDetectionConfig withCameraHeight(double newCameraHeightMeters) {
            return new BallDetectionConfig(
                cameraName,
                newCameraHeightMeters,
                targetHeightMeters,
                cameraPitchRadians,
                pipelineIndex
            );
        }

        public BallDetectionConfig withTargetHeight(double newTargetHeightMeters) {
            return new BallDetectionConfig(
                cameraName,
                cameraHeightMeters,
                newTargetHeightMeters,
                cameraPitchRadians,
                pipelineIndex
            );
        }

        public BallDetectionConfig withCameraPitch(double newCameraPitchRadians) {
            return new BallDetectionConfig(
                cameraName,
                cameraHeightMeters,
                targetHeightMeters,
                newCameraPitchRadians,
                pipelineIndex
            );
        }

        public BallDetectionConfig withPipelineIndex(int newPipelineIndex) {
            return new BallDetectionConfig(
                cameraName,
                cameraHeightMeters,
                targetHeightMeters,
                cameraPitchRadians,
                newPipelineIndex
            );
        }
    }
}
