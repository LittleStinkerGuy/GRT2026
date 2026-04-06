package frc.robot.subsystems.FMS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
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
    private RobotStatus robotStatus;

    private static final double AUTO_END = 20.0; // Auto ends at 0:20
    private static final double TRANSITION_END = 30.0; // Transition ends at 0:30
    private static final double ENDGAME_START = 130.0; // Endgame starts at 2:10 (30s before match end)
    private static final double MATCH_TOTAL = 160.0; // Total match time: 2:40
    private static final double TELEOP_SHIFT_DURATION = (ENDGAME_START - TRANSITION_END) / 6.0; // ~16.67s per shift

    // Hub activation state
    private boolean redHubActive = true;
    private boolean blueHubActive = true;
    private int currentShift = 0;
    private double timeUntilNextShift = 0.0;
    private boolean redWonAuto = false; // Set based on which alliance scored more in auto

    private NetworkTableInstance FmsNtInstance;
    private NetworkTable FmsNtTable;

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
        FmsNtInstance = NetworkTableInstance.getDefault();
        FmsNtTable = FmsNtInstance.getTable("FMSInfo");
        matchTimePublisher = FmsNtTable.getDoubleTopic("MatchTime").publish();
        isAutonomousPublisher = FmsNtTable.getBooleanTopic("IsAutonomous").publish();
        isEStoppedPublisher = FmsNtTable.getBooleanTopic("IsEStopped").publish();
        isEnabledPublisher = FmsNtTable.getBooleanTopic("IsEnabled").publish();
        isDSAttachedPublisher = FmsNtTable.getBooleanTopic("IsDSAttached").publish();

        matchStatusPublisher = FmsNtTable.getStringTopic("MatchStatus").publish();
        periodInfoPublisher = FmsNtTable.getStringTopic("PeriodInfo").publish();
        timeUntilNextPhasePublisher = FmsNtTable.getDoubleTopic("TimeUntilNextPhase").publish();
        redHubActivePublisher = FmsNtTable.getBooleanTopic("RedHubActive").publish();
        blueHubActivePublisher = FmsNtTable.getBooleanTopic("BlueHubActive").publish();
        currentShiftPublisher = FmsNtTable.getIntegerTopic("CurrentShift").publish();

        updateNetworkTables();
    }

    private void updateNetworkTables() {
        matchTimePublisher.set(DriverStation.getMatchTime());
        isAutonomousPublisher.set(DriverStation.isAutonomous());
        isEStoppedPublisher.set(DriverStation.isEStopped());
        isEnabledPublisher.set(DriverStation.isEnabled());
        isDSAttachedPublisher.set(DriverStation.isDSAttached());

        matchStatusPublisher.set(matchStatus.toString());
        periodInfoPublisher.set(periodInfo);
        timeUntilNextPhasePublisher.set(timeUntilNextPhase);
        redHubActivePublisher.set(redHubActive);
        blueHubActivePublisher.set(blueHubActive);
        currentShiftPublisher.set(currentShift + 1);
    }

    private void updateAllianceColor() {
        Alliance incomingAllianceColor = DriverStation.getAlliance().orElse(Alliance.Red);
        boolean incomingIsRed = incomingAllianceColor.equals(Alliance.Red);

        if (incomingIsRed != isRed) {
            System.out.println("Alliance color switched to " + incomingAllianceColor.toString());
            isRed = incomingIsRed;
        }
    }

    public void periodic() {
        updateAllianceColor();

        boolean incomingFMSstatus = DriverStation.isFMSAttached();
        if (incomingFMSstatus != connectedToFMS) {
            if (incomingFMSstatus) {
                System.out.println("Connected to FMS.");
            } else {
                System.out.println("Disconnected from FMS.");
            }
        }
        connectedToFMS = incomingFMSstatus;

        double matchTime = DriverStation.getMatchTime();
        double elapsedTime = MATCH_TOTAL - matchTime; // Convert remaining time to elapsed time

        if (DriverStation.isAutonomous()) {
            matchStatus = MatchStatus.AUTON;
            matchStarted = true;
            // Both hubs active during auto
            redHubActive = true;
            blueHubActive = true;
        } else if (DriverStation.isTeleop()) {
            if (elapsedTime < TRANSITION_END) {
                // Transition period (0:20 - 0:30): Both hubs still active
                matchStatus = MatchStatus.TRANSITION;
                redHubActive = true;
                blueHubActive = true;
            } else if (elapsedTime >= ENDGAME_START || matchTime <= 30) {
                // Endgame (last 30 seconds): All hubs active
                matchStatus = MatchStatus.ENDGAME;
                redHubActive = true;
                blueHubActive = true;
            } else {
                // Teleop alliance shifts (0:30 - 2:10): Alternating hub activation
                matchStatus = MatchStatus.TELEOP;
                double teleopElapsed = elapsedTime - TRANSITION_END;
                currentShift = (int) (teleopElapsed / TELEOP_SHIFT_DURATION);

                // Calculate time until next shift
                double timeInCurrentShift = teleopElapsed % TELEOP_SHIFT_DURATION;
                timeUntilNextShift = TELEOP_SHIFT_DURATION - timeInCurrentShift;

                // Determine which hub is inactive based on shift number and auto winner
                // Even shifts: loser's hub inactive, Odd shifts: winner's hub inactive
                boolean winnerInactive = (currentShift % 2 == 1);
                if (redWonAuto) {
                    redHubActive = !winnerInactive;
                    blueHubActive = winnerInactive;
                } else {
                    redHubActive = winnerInactive;
                    blueHubActive = !winnerInactive;
                }
            }
        } else if (matchTime == 0 && matchStarted) {
            matchStatus = MatchStatus.ENDED;
        }

        if (DriverStation.isEnabled()) {
            robotStatus = RobotStatus.ENABLED;
        } else if (DriverStation.isEStopped()) {
            robotStatus = RobotStatus.ESTOPPED;
        } else if (DriverStation.isDisabled()) {
            robotStatus = RobotStatus.DISABLED;
        }

        // Show period-specific time context and time until next phase
        timeUntilNextPhase = 0.0;
        if (matchStatus == MatchStatus.AUTON) {
            periodInfo = "AUTO (0:00-0:20) - All hubs active";
            timeUntilNextPhase = AUTO_END - elapsedTime;
        } else if (matchStatus == MatchStatus.TRANSITION) {
            periodInfo = "TRANSITION (0:20-0:30) - All hubs active";
            timeUntilNextPhase = TRANSITION_END - elapsedTime;
        } else if (matchStatus == MatchStatus.TELEOP) {
            periodInfo = "TELEOP Shift " + (currentShift + 1) + "/6";
            timeUntilNextPhase = timeUntilNextShift;
        } else if (matchStatus == MatchStatus.ENDGAME) {
            periodInfo = "ENDGAME (last 30s) - All hubs active";
            timeUntilNextPhase = matchTime; // Time until match end
        } else {
            periodInfo = matchStatus.toString();
        }

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
     * Sets which alliance won auto (scored more fuel).
     * This determines hub activation order during teleop shifts.
     *
     * @param redWon true if red alliance won auto
     */
    public void setAutoWinner(boolean redWon) {
        this.redWonAuto = redWon;
    }

    /**
     * Returns the current shift number (1-6) during teleop.
     *
     * @return current shift number
     */
    public int getCurrentShift() {
        return currentShift + 1;
    }
}
