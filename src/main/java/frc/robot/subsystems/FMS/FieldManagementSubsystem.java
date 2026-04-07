package frc.robot.subsystems.FMS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import java.util.Optional;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;

/** The subsystem that manages everything field related. */
public class FieldManagementSubsystem extends SubsystemBase {

    private boolean isRed;
    private boolean connectedToFMS;
    private MatchStatus matchStatus;
    private boolean matchStarted = false;
    private RobotStatus robotStatus = RobotStatus.DISABLED;

    private static final double AUTO_END = 20.0; // Auton ends at 0:20
    private static final double TRANSITION_END = 30.0; // Transition ends at 0:30
    private static final double ENDGAME_START = 130.0; // Endgame starts at 2:10 (30s before match end)
    private static final double MATCH_TOTAL = 160.0; // Total match time: 2:40
    private static final double TELEOP_SHIFT_DURATION = (ENDGAME_START - TRANSITION_END) / 4.0; // 25s per shift

    // Hub activation state
    private boolean redHubActive = true;
    private boolean blueHubActive = true;
    private int currentShift = 0;
    private double timeUntilNextShift = 0.0;
    private Optional<Boolean> redWonAuton = Optional.empty();

    private DoublePublisher matchTimePublisher;
    private BooleanPublisher isAutonomousPublisher;
    private BooleanPublisher isEStoppedPublisher;
    private BooleanPublisher isEnabledPublisher;
    private BooleanPublisher isDSAttachedPublisher;

    private StringPublisher matchStatusPublisher;
    private StringPublisher periodInfoPublisher;
    private DoublePublisher timeUntilNextPhasePublisher;
    private BooleanPublisher redHubActivePublisher;
    private BooleanPublisher blueHubActivePublisher;
    private IntegerPublisher currentShiftPublisher;

    private BooleanPublisher isFmsAttachedPublisher;

    private String periodInfo = "";
    private double timeUntilNextPhase = 0.0;

    /**
     * Initializes subsystem to handle information related to the Field Management System (such as our alliance color).
     */
    public FieldManagementSubsystem() {
        isRed = false;
        connectedToFMS = false;
        matchStatus = MatchStatus.NOT_STARTED;

        initNetworkTable();
    }

    private void initNetworkTable() {
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        NetworkTable fmsNtTable = ntInstance.getTable("FMSInfo/Extra");
        matchTimePublisher = fmsNtTable.getDoubleTopic("MatchTime").publish();
        isAutonomousPublisher = fmsNtTable.getBooleanTopic("IsAutonomous").publish();
        isEStoppedPublisher = fmsNtTable.getBooleanTopic("IsEStopped").publish();
        isEnabledPublisher = fmsNtTable.getBooleanTopic("IsEnabled").publish();
        isDSAttachedPublisher = fmsNtTable.getBooleanTopic("IsDSAttached").publish();
        isFmsAttachedPublisher = fmsNtTable.getBooleanTopic("IsFMSAttached").publish();

        matchStatusPublisher = fmsNtTable.getStringTopic("MatchStatus").publish();
        periodInfoPublisher = fmsNtTable.getStringTopic("PeriodInfo").publish();
        timeUntilNextPhasePublisher = fmsNtTable.getDoubleTopic("TimeUntilNextPhase").publish();
        redHubActivePublisher = fmsNtTable.getBooleanTopic("RedHubActive").publish();
        blueHubActivePublisher = fmsNtTable.getBooleanTopic("BlueHubActive").publish();
        currentShiftPublisher = fmsNtTable.getIntegerTopic("CurrentShift").publish();

        updateNetworkTables();
    }

    private void updateNetworkTables() {
        matchTimePublisher.set(DriverStation.getMatchTime());
        // isAutonomousPublisher.set(DriverStation.isAutonomous());
        // isEStoppedPublisher.set(DriverStation.isEStopped());
        // isEnabledPublisher.set(DriverStation.isEnabled());
        // isDSAttachedPublisher.set(DriverStation.isDSAttached());
        // isFmsAttachedPublisher.set(DriverStation.isFMSAttached());

        // matchStatusPublisher.set(matchStatus.toString());
        // periodInfoPublisher.set(periodInfo);
        timeUntilNextPhasePublisher.set(timeUntilNextPhase);
        redHubActivePublisher.set(redHubActive);
        // blueHubActivePublisher.set(blueHubActive);
        currentShiftPublisher.set(getCurrentShift());
    }

    private Optional<Boolean> didRedWinAuton() {
        if (redWonAuton.isPresent()) {
            return redWonAuton;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) {
            return Optional.empty();
        }

        redWonAuton = switch (gameData.charAt(0)) {
            case 'R' -> Optional.of(true);
            case 'B' -> Optional.of(false);
            default -> Optional.empty();
        };

        return redWonAuton;
    }

    private void updateAllianceColor() {
        Alliance incomingAllianceColor = DriverStation.getAlliance().orElse(Alliance.Red);
        boolean incomingIsRed = incomingAllianceColor.equals(Alliance.Red);

        if (incomingIsRed != isRed) {
            System.out.println("Alliance color switched to " + incomingAllianceColor.toString());
            isRed = incomingIsRed;
        }
    }

    private void updateFmsStatus() {
        boolean incomingFMSstatus = DriverStation.isFMSAttached();
        if (incomingFMSstatus != connectedToFMS) {
            if (incomingFMSstatus) {
                System.out.println("Connected to FMS.");
            } else {
                System.out.println("Disconnected from FMS.");
            }
            connectedToFMS = incomingFMSstatus;
        }
    }

    private void updateRobotStatus() {
        if (DriverStation.isEStopped()) {
            robotStatus = RobotStatus.ESTOPPED;
        } else if (DriverStation.isEnabled()) {
            robotStatus = RobotStatus.ENABLED;
        } else {
            robotStatus = RobotStatus.DISABLED;
        }
    }

    private void updateMatchStates(double matchTime) {
        matchTime = Math.max(matchTime, 0);
        double elapsedTime = MATCH_TOTAL - matchTime; // Convert remaining time to elapsed time

        if (DriverStation.isAutonomous()) {
            matchStatus = MatchStatus.AUTON;
            matchStarted = true;
            // Both hubs active during auton
            redHubActive = true;
            blueHubActive = true;
        } else if (DriverStation.isTeleop()) {
            if (elapsedTime < TRANSITION_END) {
                // Transition period (0:20 - 0:30): Both hubs still active
                matchStatus = MatchStatus.TRANSITION;
                redHubActive = true;
                blueHubActive = true;
            } else if (matchTime <= 30) {
                // Endgame (last 30 seconds): All hubs active
                matchStatus = MatchStatus.ENDGAME;
                redHubActive = true;
                blueHubActive = true;
            } else {
                // Teleop alliance shifts (0:30 - 2:10): Alternating hub activation
                matchStatus = MatchStatus.TELEOP;
                double teleopElapsed = elapsedTime - TRANSITION_END;
                currentShift = Math.min((int) (teleopElapsed / TELEOP_SHIFT_DURATION), 3);

                // Calculate time until next shift
                double timeInCurrentShift = teleopElapsed % TELEOP_SHIFT_DURATION;
                timeUntilNextShift = TELEOP_SHIFT_DURATION - timeInCurrentShift;

                // Determine which hub is inactive based on shift number and auton winner
                // 0-indexed: Even shifts: winners's hub inactive, Odd shifts: losers's hub inactive
                boolean winnerActive = (currentShift % 2 == 1);
                if (didRedWinAuton().isEmpty()) { // ideally doesn't happen but yk idk what behavior should happen here
                    redHubActive = true;
                    blueHubActive = true;
                } else {
                    redHubActive = didRedWinAuton().get() ? winnerActive : !winnerActive;
                    blueHubActive = didRedWinAuton().get() ? !winnerActive : winnerActive;
                }
            }
        } else if (matchTime <= 0.0 && matchStarted) {
            matchStatus = MatchStatus.ENDED;
        }
    }

    private void updatePeriodInfo(double matchTime) {
        double elapsedTime = MATCH_TOTAL - matchTime; // Convert remaining time to elapsed time

        // Show period-specific time context and time until next phase
        timeUntilNextPhase = 0.0;
        if (matchStatus == MatchStatus.AUTON) {
            periodInfo = "AUTO (0:00-0:20) - All hubs active";
            timeUntilNextPhase = AUTO_END - elapsedTime;
        } else if (matchStatus == MatchStatus.TRANSITION) {
            periodInfo = "TRANSITION (0:20-0:30) - All hubs active";
            timeUntilNextPhase = TRANSITION_END - elapsedTime;
        } else if (matchStatus == MatchStatus.TELEOP) {
            periodInfo = "TELEOP Shift " + (currentShift + 1) + "/4";
            timeUntilNextPhase = timeUntilNextShift;
        } else if (matchStatus == MatchStatus.ENDGAME) {
            periodInfo = "ENDGAME (last 30s) - All hubs active";
            timeUntilNextPhase = matchTime; // Time until match end
        } else {
            periodInfo = matchStatus.toString();
        }
    }

    public void periodic() {
        updateAllianceColor();
        updateFmsStatus();
        updateRobotStatus();

        double matchTime = DriverStation.getMatchTime();
        updateMatchStates(matchTime);
        updatePeriodInfo(matchTime);

        updateNetworkTables();
    }

    /**
     * Identifies whether or not we are Red Alliance.
     * 
     * @return isRed boolean
     */
    public boolean isRedAlliance() {
        return isRed;
    }

    /**
     * Identifies whether or not we are connected to an FRC Field Management System.
     * 
     * @return connectedToFMS boolean
     */
    public boolean isConnectedToFMS() {
        return connectedToFMS;
    }

    /**
     * Returns current match status (AUTON, TELEOP, ENDGAME).
     * 
     * @return current MatchStatus
     */
    public MatchStatus getMatchStatus() {
        return matchStatus;
    }

    /**
     * Returns current robot status (ENABLED, DISABLED, ESTOPPED).
     *
     * @return current robot status
     */
    public RobotStatus getRobotStatus() {
        return robotStatus;
    }

    /**
     * Returns whether the red hub is currently active.
     *
     * @return true if red hub is active
     */
    public boolean isRedHubActive() {
        return redHubActive;
    }

    /**
     * Returns whether the blue hub is currently active.
     *
     * @return true if blue hub is active
     */
    public boolean isBlueHubActive() {
        return blueHubActive;
    }

    /**
     * Returns whether our alliance's hub is currently active.
     *
     * @return true if our hub is active
     */
    public boolean isOurHubActive() {
        return isRed ? redHubActive : blueHubActive;
    }

    /**
     * Returns the current shift number (1-4) during teleop.
     *
     * @return current shift number
     */
    public int getCurrentShift() {
        return currentShift + 1;
    }
}
