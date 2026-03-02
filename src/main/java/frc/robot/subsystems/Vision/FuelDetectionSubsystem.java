package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

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
            Optional<Distance> distanceMeters) {
    }

    private final PhotonCamera camera;
    private int pipelineIndex;
    private final Distance cameraHeight;
    private final Distance targetHeight;
    private final Angle cameraPitch;

    private final String dashboardPrefix;

    private Consumer<Detection> detectionConsumer = NO_OP_CONSUMER;

    private List<Detection> latestDetections = List.of();
    private Optional<Detection> bestDetection = Optional.empty();

    private final Deque<Distance> distanceWindow = new ArrayDeque<>();
    private Distance distanceWindowSum = Meters.of(0.0);

    private final Deque<Distance> minDistanceWindow = new ArrayDeque<>();
    private Distance minDistanceWindowSum = Meters.of(0.0);

    private final Deque<Distance> maxDistanceWindow = new ArrayDeque<>();
    private Distance maxDistanceWindowSum = Meters.of(0.0);

    private Optional<Distance> filteredDistance = Optional.empty();
    private Optional<Distance> filteredMinDistance = Optional.empty();
    private Optional<Distance> filteredMaxDistance = Optional.empty();
    private Optional<Time> latestTimestamp = Optional.empty();
    private Optional<Time> startDecayTime = Optional.empty();

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
        cameraHeight = config.cameraHeight();
        targetHeight = config.targetHeight();
        cameraPitch = config.cameraPitch();
        dashboardPrefix = "FuelDetection/" + config.cameraName() + "/";
        camera.setPipelineIndex(pipelineIndex);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        Time timeNow = Seconds.of(Timer.getFPGATimestamp());

        if (results.isEmpty()) {
            applyDecayToFilteredValues(timeNow);
            publishTelemetry();
            return;
        }

        for (PhotonPipelineResult result : results) {
            handleResult(result, timeNow);
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

    private void handleResult(PhotonPipelineResult result, Time robotTimestamp) {
        List<Detection> processed = new ArrayList<>();

        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                processed.add(createDetection(result.getTimestampSeconds(), target));
            }
        }

        latestDetections = List.copyOf(processed);
        if (processed.isEmpty()) {
            bestDetection = Optional.empty();
            applyDecayToFilteredValues(robotTimestamp);
        } else {
            bestDetection = processed.stream()
                    .max(Comparator.comparingDouble(Detection::area));
            Optional<Distance> minDistance = findDistanceExtreme(processed, true);
            Optional<Distance> maxDistance = findDistanceExtreme(processed, false);
            bestDetection.ifPresentOrElse(
                    detection -> {
                        recordDetectionForSmoothing(detection, minDistance, maxDistance, robotTimestamp);
                        detectionConsumer.accept(detection);
                    },
                    () -> applyDecayToFilteredValues(robotTimestamp));
        }
    }

    private Detection createDetection(double timestampSeconds, PhotonTrackedTarget target) {
        Optional<Distance> distanceMeters = Optional.empty();
        double calculatedMeters = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight.in(Meters),
                targetHeight.in(Meters),
                cameraPitch.in(Radians),
                Units.degreesToRadians(target.getPitch()));
        if (Double.isFinite(calculatedMeters)) {
            distanceMeters = Optional.of(Meters.of(calculatedMeters));
        }
        return new Detection(
                timestampSeconds,
                target.getYaw(),
                target.getPitch(),
                target.getArea(),
                distanceMeters);
    }

    private void recordDetectionForSmoothing(
            Detection detection,
            Optional<Distance> minDistance,
            Optional<Distance> maxDistance,
            Time robotTimestamp) {
        detection.distanceMeters().ifPresent(detectionDistance ->
                updateSmoothedDistance(DistanceSampleType.BEST, detectionDistance));
        minDistance.ifPresent(distance -> updateSmoothedDistance(DistanceSampleType.MIN, distance));
        maxDistance.ifPresent(distance -> updateSmoothedDistance(DistanceSampleType.MAX, distance));
        latestTimestamp = Optional.of(Seconds.of(detection.timestampSeconds()));
        startDecayTime = Optional.of(robotTimestamp.plus(VisionConstants.FUEL_DECAY_HOLD_TIME_SECONDS));
    }

    private void applyDecayToFilteredValues(Time timeNow) {
        if (startDecayTime.isEmpty()) {
            resetFilteredState();
            return;
        }
        Time decayStartTime = startDecayTime.get();
        if (timeNow.lte(decayStartTime)) {
            return;
        }

        Time elapsed = timeNow.minus(decayStartTime);
        double decayProgress = Math.min(elapsed.div(VisionConstants.FUEL_DECAY_TIME_SECONDS).in(Value), 1.0);
        double scale = Math.max(0.0, 1.0 - decayProgress);

        filteredDistance = filteredDistance.map(
                distance -> Meters.of(distance.in(Meters) * scale));
        filteredMinDistance = filteredMinDistance.map(
                distance -> Meters.of(distance.in(Meters) * scale));
        filteredMaxDistance = filteredMaxDistance.map(
                distance -> Meters.of(distance.in(Meters) * scale));

        if (decayProgress >= 1.0) {
            resetFilteredState();
            startDecayTime = Optional.empty();
        }
    }

    private void resetFilteredState() {
        filteredDistance = Optional.empty();
        filteredMinDistance = Optional.empty();
        filteredMaxDistance = Optional.empty();
        latestTimestamp = Optional.empty();
        distanceWindow.clear();
        distanceWindowSum = Meters.of(0.0);
        minDistanceWindow.clear();
        minDistanceWindowSum = Meters.of(0.0);
        maxDistanceWindow.clear();
        maxDistanceWindowSum = Meters.of(0.0);
    }

    private Distance appendSample(Deque<Distance> window, Distance value, Distance currentSum) {
        window.addLast(value);
        double sumMeters = currentSum.in(Meters) + value.in(Meters);
        if (window.size() > VisionConstants.FUEL_SMOOTHING_WINDOW_SIZE) {
            sumMeters -= window.removeFirst().in(Meters);
        }
        return Meters.of(sumMeters);
    }

    private void updateSmoothedDistance(DistanceSampleType sampleType, Distance distance) {
        switch (sampleType) {
            case BEST -> {
                distanceWindowSum = appendSample(distanceWindow, distance, distanceWindowSum);
                double averageMeters = distanceWindowSum.in(Meters) / distanceWindow.size();
                filteredDistance = Optional.of(Meters.of(averageMeters));
            }
            case MIN -> {
                minDistanceWindowSum = appendSample(minDistanceWindow, distance, minDistanceWindowSum);
                double averageMeters = minDistanceWindowSum.in(Meters) / minDistanceWindow.size();
                filteredMinDistance = Optional.of(Meters.of(averageMeters));
            }
            case MAX -> {
                maxDistanceWindowSum = appendSample(maxDistanceWindow, distance, maxDistanceWindowSum);
                double averageMeters = maxDistanceWindowSum.in(Meters) / maxDistanceWindow.size();
                filteredMaxDistance = Optional.of(Meters.of(averageMeters));
            }
        }
    }

    private Optional<Distance> findDistanceExtreme(List<Detection> detections, boolean findMin) {
        Distance bestDistance = null;
        for (Detection detection : detections) {
            Optional<Distance> distance = detection.distanceMeters();
            if (distance.isEmpty()) {
                continue;
            }
            Distance candidate = distance.get();
            if (bestDistance == null) {
                bestDistance = candidate;
                continue;
            }
            double candidateMeters = candidate.in(Meters);
            double bestMeters = bestDistance.in(Meters);
            if (findMin ? candidateMeters < bestMeters : candidateMeters > bestMeters) {
                bestDistance = candidate;
            }
        }
        return Optional.ofNullable(bestDistance);
    }

    public Optional<Distance> getClosestDistance(){
        return filteredMinDistance;
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber(key("count"), latestDetections.size());

        double[] yawSamples = latestDetections.stream().mapToDouble(Detection::yawDegrees).toArray();
        double[] pitchSamples = latestDetections.stream().mapToDouble(Detection::pitchDegrees).toArray();

        SmartDashboard.putNumberArray(key("yawSamples"), yawSamples);
        SmartDashboard.putNumberArray(key("pitchSamples"), pitchSamples);

        double[] distanceSamples = latestDetections.stream()
                .map(Detection::distanceMeters)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .mapToDouble(distance -> distance.in(Meters))
                .toArray();
        SmartDashboard.putNumberArray(key("distanceSamples"), distanceSamples);

        SmartDashboard.putNumber(key("bestYawDeg"), bestDetection.orElse(EMPTY_DETECTION).yawDegrees());
        SmartDashboard.putNumber(key("bestPitchDeg"), bestDetection.orElse(EMPTY_DETECTION).pitchDegrees());
        SmartDashboard.putNumber(key("bestDistanceMeters"),
                filteredDistance.orElse(Meters.of(Double.NaN)).in(Meters));
        SmartDashboard.putNumber(key("minDistanceMeters"),
                filteredMinDistance.orElse(Meters.of(Double.NaN)).in(Meters));
        SmartDashboard.putNumber(key("maxDistanceMeters"),
                filteredMaxDistance.orElse(Meters.of(Double.NaN)).in(Meters));
        SmartDashboard.putNumber(key("timestampSeconds"),
                latestTimestamp.orElse(Seconds.of(Double.NaN)).in(Seconds));
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
            Distance cameraHeight,
            Distance targetHeight,
            Angle cameraPitch,
            int pipelineIndex) {
        public FuelDetectionConfig {
            Objects.requireNonNull(cameraName, "cameraName is required");
            Objects.requireNonNull(cameraHeight, "cameraHeight is required");
            Objects.requireNonNull(targetHeight, "targetHeight is required");
            Objects.requireNonNull(cameraPitch, "cameraPitch is required");
            if (!Double.isFinite(cameraHeight.in(Meters))) {
                throw new IllegalArgumentException("cameraHeight must be finite");
            }
            if (!Double.isFinite(targetHeight.in(Meters))) {
                throw new IllegalArgumentException("targetHeight must be finite");
            }
            if (!Double.isFinite(cameraPitch.in(Radians))) {
                throw new IllegalArgumentException("cameraPitch must be finite");
            }
        }

        /**
         * Creates a default configuration for the supplied camera.
         */
        public static FuelDetectionConfig defaultConfig(
                String cameraName,
                Distance cameraHeight,
                Distance targetHeight,
                Angle cameraPitch) {
            return new FuelDetectionConfig(
                    cameraName,
                    cameraHeight,
                    targetHeight,
                    cameraPitch,
                    0);
        }

        public FuelDetectionConfig withCameraHeight(Distance newCameraHeight) {
            return new FuelDetectionConfig(
                    cameraName,
                    newCameraHeight,
                    targetHeight,
                    cameraPitch,
                    pipelineIndex);
        }

        public FuelDetectionConfig withTargetHeight(Distance newTargetHeight) {
            return new FuelDetectionConfig(
                    cameraName,
                    cameraHeight,
                    newTargetHeight,
                    cameraPitch,
                    pipelineIndex);
        }

        public FuelDetectionConfig withCameraPitch(Angle newCameraPitch) {
            return new FuelDetectionConfig(
                    cameraName,
                    cameraHeight,
                    targetHeight,
                    newCameraPitch,
                    pipelineIndex);
        }

        public FuelDetectionConfig withPipelineIndex(int newPipelineIndex) {
            return new FuelDetectionConfig(
                    cameraName,
                    cameraHeight,
                    targetHeight,
                    cameraPitch,
                    newPipelineIndex);
        }
    }
}
