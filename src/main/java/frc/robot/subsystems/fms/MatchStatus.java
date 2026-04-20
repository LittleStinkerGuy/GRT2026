package frc.robot.subsystems.fms;

/**
 * Holds current state of match.
 */
public enum MatchStatus {
    NOT_STARTED,
    AUTON,
    TRANSITION,
    TELEOP,
    ENDGAME,
    ENDED;
}
