package frc.robot.subsystems.FMS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

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

    private NetworkTableInstance FMSNTInstance;
    private NetworkTable FMSNTTable;
    private NetworkTableEntry stationNumberEntry;
    private NetworkTableEntry matchNumberEntry;
    private NetworkTableEntry matchTypeEntry;
    private NetworkTableEntry timeLeftEntry;
    private NetworkTableEntry isAutonomousEntry;
    private NetworkTableEntry isEStoppedEntry;
    private NetworkTableEntry isEnabledEntry;
    private NetworkTableEntry isDSAttachedEntry;

    /**
     * Initializes subsystem to handle information related to the Field Management System (such as our alliance color).
     */
    public FieldManagementSubsystem() {

        isRed = false;
        connectedToFMS = false;
        matchStatus = MatchStatus.NOTSTARTED;

        FMSNTInstance = NetworkTableInstance.getDefault();
        FMSNTTable = FMSNTInstance.getTable("FMS");
        stationNumberEntry = FMSNTTable.getEntry("StationNumber");
        matchNumberEntry = FMSNTTable.getEntry("MatchNumber");
        matchTypeEntry = FMSNTTable.getEntry("MatchType");
        timeLeftEntry = FMSNTTable.getEntry("TimeLeft");
        isAutonomousEntry = FMSNTTable.getEntry("IsAutonomous");
        isEStoppedEntry = FMSNTTable.getEntry("IsEStopped");
        isEnabledEntry = FMSNTTable.getEntry("IsEnabled");
        isDSAttachedEntry = FMSNTTable.getEntry("IsDSAttached");
    }

    public void periodic() {
        boolean incomingIsRed;
        try {
            incomingIsRed = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
        } catch (Exception e) {
            incomingIsRed = isRed;
        }

        if (incomingIsRed != isRed) {
            if (incomingIsRed) {
                System.out.println("Alliance color switched to Red.");
            } else {
                System.out.println("Alliance color switched to Blue.");
            }
        }
        isRed = incomingIsRed;

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

        stationNumberEntry.setInteger(DriverStation.getLocation().orElse(-1));
        matchNumberEntry.setInteger(DriverStation.getMatchNumber());
        matchTypeEntry.setString(DriverStation.getMatchType().toString());
        timeLeftEntry.setDouble(DriverStation.getMatchTime());
        isAutonomousEntry.setBoolean(DriverStation.isAutonomous());
        isEStoppedEntry.setBoolean(DriverStation.isEStopped());
        isEnabledEntry.setBoolean(DriverStation.isEnabled());
        isDSAttachedEntry.setBoolean(DriverStation.isDSAttached());

        // Game timer
        // 20s Auto + 10s Transition + 100s Teleop (6 shifts) + 30s Endgame = 160s total
        SmartDashboard.putNumber("Match Time Remaining", matchTime);
        SmartDashboard.putString("Match Period", matchStatus.toString());

        int minutes = (int) (matchTime / 60);
        int seconds = (int) (matchTime % 60);
        String formattedTime = String.format("%d:%02d", minutes, seconds);
        SmartDashboard.putString("Match Timer", formattedTime);

        // Show period-specific time context and time until next phase
        double timeUntilNextPhase = 0.0;
        if (matchStatus == MatchStatus.AUTON) {
            SmartDashboard.putString("Period Info", "AUTO (0:00-0:20) - All hubs active");
            timeUntilNextPhase = AUTO_END - elapsedTime;
        } else if (matchStatus == MatchStatus.TRANSITION) {
            SmartDashboard.putString("Period Info", "TRANSITION (0:20-0:30) - All hubs active");
            timeUntilNextPhase = TRANSITION_END - elapsedTime;
        } else if (matchStatus == MatchStatus.TELEOP) {
            SmartDashboard.putString("Period Info", "TELEOP Shift " + (currentShift + 1) + "/6");
            timeUntilNextPhase = timeUntilNextShift;
        } else if (matchStatus == MatchStatus.ENDGAME) {
            SmartDashboard.putString("Period Info", "ENDGAME (last 30s) - All hubs active");
            timeUntilNextPhase = matchTime; // Time until match end
        } else {
            SmartDashboard.putString("Period Info", matchStatus.toString());
        }

        // Display time until next phase shift
        SmartDashboard.putNumber("Time Until Next Phase", timeUntilNextPhase);
        String nextPhaseFormatted = String.format("%.1fs", timeUntilNextPhase);
        SmartDashboard.putString("Next Phase In", nextPhaseFormatted);

        // Hub activation status
        SmartDashboard.putBoolean("Red Hub Active", redHubActive);
        SmartDashboard.putBoolean("Blue Hub Active", blueHubActive);
        SmartDashboard.putNumber("Current Shift", currentShift + 1);
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
