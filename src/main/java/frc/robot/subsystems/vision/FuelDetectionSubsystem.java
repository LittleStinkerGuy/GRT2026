package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public class FuelDetectionSubsystem extends SubsystemBase {

    private static final Detection EMPTY_DETECTION = new Detection(
        Double.NaN,
        Double.NaN,
        Double.NaN,
        Double.NaN,
        Optional.empty());

    private static final Consumer<Detection> NO_OP_CONSUMER = detection -> {
    };

    public record Detection(
        double timestampSeconds,
        double yawDegrees,
        double pitchDegrees,
        double area,
        Optional<Double> distanceM) {}

    private final PhotonCamera camera;
    private int pipelineIndex;
    private final double cameraHeightM;
    private final double targetHeightM;
    private final double cameraPitchRad;

    private final String dashboardPrefix;

    private Consumer<Detection> detectionConsumer = NO_OP_CONSUMER;

    private List<Detection> latestDetections = List.of();
    private Optional<Detection> bestDetection = Optional.empty();

    private final Deque<Double> distanceWindow = new ArrayDeque<>();
    private double distanceWindowSumM = 0.0;

    private final Deque<Double> minDistanceWindow = new ArrayDeque<>();
    private double minDistanceWindowSumM = 0.0;

    private final Deque<Double> maxDistanceWindow = new ArrayDeque<>();
    private double maxDistanceWindowSumM = 0.0;

    private Optional<Double> filteredDistanceM = Optional.empty();
    private Optional<Double> filteredMinDistanceM = Optional.empty();
    private Optional<Double> filteredMaxDistanceM = Optional.empty();
    private Optional<Double> latestTimestampSec = Optional.empty();
    private Optional<Double> startDecayTimeSec = Optional.empty();

    private enum DistanceSampleType {
        BEST,
        MIN,
        MAX
    }

    /**
     * Creates the subsystem with a custom configuration.
     *
     * @param config user supplied configuration
     */
    public FuelDetectionSubsystem(FuelDetectionConfig config) {
        Objects.requireNonNull(config, "FuelDetectionConfig cannot be null");

        camera = new PhotonCamera(config.cameraName());
        pipelineIndex = config.pipelineIndex();
        cameraHeightM = config.cameraHeightM();
        targetHeightM = config.targetHeightM();
        cameraPitchRad = config.cameraPitchRad();
        dashboardPrefix = "FuelDetection/" + config.cameraName() + "/";
        camera.setPipelineIndex(pipelineIndex);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        double timeNowSec = Timer.getFPGATimestamp();

        if (results.isEmpty()) {
            applyDecayToFilteredValues(timeNowSec);
            publishTelemetry();
            return;
        }

        for (PhotonPipelineResult result : results) {
            handleResult(result, timeNowSec);
        }
        publishTelemetry();
    }

    /**
     * Assigns the consumer that should be notified whenever a new best detection is
     * produced. Passing {@code null} clears the consumer.
     */
    public void setBestDetectionConsumer(Consumer<Detection> consumer) {
        detectionConsumer = consumer != null ? consumer : NO_OP_CONSUMER;
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

    public boolean isBallDetected() {
        return getClosestDistance().isPresent();
    }

    private void handleResult(PhotonPipelineResult result, double robotTimestampSec) {
        List<Detection> processed = new ArrayList<>();

        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                processed.add(createDetection(result.getTimestampSeconds(), target));
            }
        }

        latestDetections = List.copyOf(processed);
        if (processed.isEmpty()) {
            bestDetection = Optional.empty();
            applyDecayToFilteredValues(robotTimestampSec);
        } else {
            bestDetection = processed.stream()
                .max(Comparator.comparingDouble(Detection::area));
            Optional<Double> minDistance = findDistanceExtreme(processed, true);
            Optional<Double> maxDistance = findDistanceExtreme(processed, false);
            bestDetection.ifPresentOrElse(
                detection -> {
                    recordDetectionForSmoothing(detection, minDistance, maxDistance, robotTimestampSec);
                    detectionConsumer.accept(detection);
                },
                () -> applyDecayToFilteredValues(robotTimestampSec));
        }
    }

    private Detection createDetection(double timestampSeconds, PhotonTrackedTarget target) {
        Optional<Double> distanceM = Optional.empty();
        double calculatedMeters = PhotonUtils.calculateDistanceToTargetMeters(
            cameraHeightM,
            targetHeightM,
            cameraPitchRad,
            Units.degreesToRadians(target.getPitch()));
        if (Double.isFinite(calculatedMeters)) {
            distanceM = Optional.of(calculatedMeters);
        }
        return new Detection(
            timestampSeconds,
            target.getYaw(),
            target.getPitch(),
            target.getArea(),
            distanceM);
    }

    private void recordDetectionForSmoothing(
        Detection detection,
        Optional<Double> minDistanceM,
        Optional<Double> maxDistanceM,
        double robotTimestampSec) {
        detection.distanceM().ifPresent(detectionDistance -> updateSmoothedDistance(DistanceSampleType.BEST, detectionDistance));
        minDistanceM.ifPresent(distance -> updateSmoothedDistance(DistanceSampleType.MIN, distance));
        maxDistanceM.ifPresent(distance -> updateSmoothedDistance(DistanceSampleType.MAX, distance));
        latestTimestampSec = Optional.of(detection.timestampSeconds());
        startDecayTimeSec = Optional.of(robotTimestampSec + VisionConstants.FUEL_DECAY_HOLD_TIME_SEC);
    }

    private void applyDecayToFilteredValues(double timeNowSec) {
        if (startDecayTimeSec.isEmpty()) {
            resetFilteredState();
            return;
        }
        double decayStartTimeSec = startDecayTimeSec.get();
        if (timeNowSec <= decayStartTimeSec) {
            return;
        }

        double elapsedSec = timeNowSec - decayStartTimeSec;
        double decayProgress = Math.min(elapsedSec / VisionConstants.FUEL_DECAY_TIME_SEC, 1.0);
        double scale = Math.max(0.0, 1.0 - decayProgress);

        filteredDistanceM = filteredDistanceM.map(distance -> distance * scale);
        filteredMinDistanceM = filteredMinDistanceM.map(distance -> distance * scale);
        filteredMaxDistanceM = filteredMaxDistanceM.map(distance -> distance * scale);

        if (decayProgress >= 1.0) {
            resetFilteredState();
            startDecayTimeSec = Optional.empty();
        }
    }

    private void resetFilteredState() {
        filteredDistanceM = Optional.empty();
        filteredMinDistanceM = Optional.empty();
        filteredMaxDistanceM = Optional.empty();
        latestTimestampSec = Optional.empty();
        distanceWindow.clear();
        distanceWindowSumM = 0.0;
        minDistanceWindow.clear();
        minDistanceWindowSumM = 0.0;
        maxDistanceWindow.clear();
        maxDistanceWindowSumM = 0.0;
    }

    private double appendSample(Deque<Double> window, double valueM, double currentSumM) {
        window.addLast(valueM);
        double sumMeters = currentSumM + valueM;
        if (window.size() > VisionConstants.FUEL_SMOOTHING_WINDOW_SIZE) {
            sumMeters -= window.removeFirst();
        }
        return sumMeters;
    }

    private void updateSmoothedDistance(DistanceSampleType sampleType, double distanceM) {
        switch (sampleType) {
            case BEST -> {
                distanceWindowSumM = appendSample(distanceWindow, distanceM, distanceWindowSumM);
                double averageMeters = distanceWindowSumM / distanceWindow.size();
                filteredDistanceM = Optional.of(averageMeters);
            }
            case MIN -> {
                minDistanceWindowSumM = appendSample(minDistanceWindow, distanceM, minDistanceWindowSumM);
                double averageMeters = minDistanceWindowSumM / minDistanceWindow.size();
                filteredMinDistanceM = Optional.of(averageMeters);
            }
            case MAX -> {
                maxDistanceWindowSumM = appendSample(maxDistanceWindow, distanceM, maxDistanceWindowSumM);
                double averageMeters = maxDistanceWindowSumM / maxDistanceWindow.size();
                filteredMaxDistanceM = Optional.of(averageMeters);
            }
            default -> {
            }
        }
    }

    private Optional<Double> findDistanceExtreme(List<Detection> detections, boolean findMin) {
        Double bestDistanceM = null;
        for (Detection detection : detections) {
            Optional<Double> distance = detection.distanceM();
            if (distance.isEmpty()) {
                continue;
            }
            double candidateM = distance.get();
            if (bestDistanceM == null) {
                bestDistanceM = candidateM;
                continue;
            }
            if (findMin ? candidateM < bestDistanceM : candidateM > bestDistanceM) {
                bestDistanceM = candidateM;
            }
        }
        return Optional.ofNullable(bestDistanceM);
    }

    public Optional<Double> getClosestDistance() {
        return filteredMinDistanceM;
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber(key("count"), latestDetections.size());

        double[] yawSamples = latestDetections.stream().mapToDouble(Detection::yawDegrees).toArray();
        double[] pitchSamples = latestDetections.stream().mapToDouble(Detection::pitchDegrees).toArray();

        SmartDashboard.putNumberArray(key("yawSamples"), yawSamples);
        SmartDashboard.putNumberArray(key("pitchSamples"), pitchSamples);

        double[] distanceSamples = latestDetections.stream()
            .map(Detection::distanceM)
            .filter(Optional::isPresent)
            .mapToDouble(Optional::get)
            .toArray();
        SmartDashboard.putNumberArray(key("distanceSamples"), distanceSamples);

        SmartDashboard.putNumber(key("bestYawDeg"), bestDetection.orElse(EMPTY_DETECTION).yawDegrees());
        SmartDashboard.putNumber(key("bestPitchDeg"), bestDetection.orElse(EMPTY_DETECTION).pitchDegrees());
        SmartDashboard.putNumber(key("bestDistanceMeters"),
            filteredDistanceM.orElse(Double.NaN));
        SmartDashboard.putNumber(key("minDistanceMeters"),
            filteredMinDistanceM.orElse(Double.NaN));
        SmartDashboard.putNumber(key("maxDistanceMeters"),
            filteredMaxDistanceM.orElse(Double.NaN));
        SmartDashboard.putNumber(key("timestampSeconds"),
            latestTimestampSec.orElse(Double.NaN));
    }

    private String key(String suffix) {
        return dashboardPrefix + suffix;
    }

    /**
     * Configuration record used to describe the colored-shape pipeline setup.
     * Values must be finite. Shoutout Codex for ts code
     */
    public static record FuelDetectionConfig(
        String cameraName,
        double cameraHeightM,
        double targetHeightM,
        double cameraPitchRad,
        int pipelineIndex) {
        public FuelDetectionConfig {
            Objects.requireNonNull(cameraName, "cameraName is required");
            if (!Double.isFinite(cameraHeightM)) {
                throw new IllegalArgumentException("cameraHeightM must be finite");
            }
            if (!Double.isFinite(targetHeightM)) {
                throw new IllegalArgumentException("targetHeightM must be finite");
            }
            if (!Double.isFinite(cameraPitchRad)) {
                throw new IllegalArgumentException("cameraPitchRad must be finite");
            }
        }

        /**
         * Creates a default configuration for the supplied camera.
         */
        public static FuelDetectionConfig defaultConfig(
            String cameraName,
            double cameraHeightM,
            double targetHeightM,
            double cameraPitchRad) {
            return new FuelDetectionConfig(
                cameraName,
                cameraHeightM,
                targetHeightM,
                cameraPitchRad,
                0);
        }

        public FuelDetectionConfig withCameraHeight(double newCameraHeightM) {
            return new FuelDetectionConfig(
                cameraName,
                newCameraHeightM,
                targetHeightM,
                cameraPitchRad,
                pipelineIndex);
        }

        public FuelDetectionConfig withTargetHeight(double newTargetHeightM) {
            return new FuelDetectionConfig(
                cameraName,
                cameraHeightM,
                newTargetHeightM,
                cameraPitchRad,
                pipelineIndex);
        }

        public FuelDetectionConfig withCameraPitch(double newCameraPitchRad) {
            return new FuelDetectionConfig(
                cameraName,
                cameraHeightM,
                targetHeightM,
                newCameraPitchRad,
                pipelineIndex);
        }

        public FuelDetectionConfig withPipelineIndex(int newPipelineIndex) {
            return new FuelDetectionConfig(
                cameraName,
                cameraHeightM,
                targetHeightM,
                cameraPitchRad,
                newPipelineIndex);
        }
    }
}
