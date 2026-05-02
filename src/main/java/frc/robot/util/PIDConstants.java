package frc.robot.util;

/**
 * Bag of PID + feedforward gains shared across IO implementations. Used to
 * seed dashboard tuning entries (see {@link LoggedTunableNumber}).
 * Each IO impl returns its own preferred values from a {@code getDefaultPID()}
 * hook so sim plants can ship sim-specific gains without leaking into shared
 * constants.
 *
 * <p>
 * Construct via {@link #ZERO} plus {@code withX} setters so call sites name
 * each gain explicitly and only mention the non-zero ones, e.g.
 * {@code PIDConstants.ZERO.withKP(0.5).withKV(0.3)}.
 *
 * <p>
 * Component meanings (units depend on the controller — Phoenix 6 slot
 * configs use volts per error unit; WPILib PIDController uses output units):
 *
 * @param kP proportional gain — output per unit of error
 * @param kI integral gain — output per (error · second)
 * @param kD derivative gain — output per (error / second)
 * @param kS static-friction feedforward — constant output added in the
 *        direction of motion to overcome stiction
 * @param kG gravity feedforward — constant output to counteract gravity
 *        (arms / elevators); leave 0 for non-gravity-loaded mechanisms
 * @param kV velocity feedforward — output per unit of velocity setpoint
 * @param kA acceleration feedforward — output per unit of acceleration setpoint
 */
public record PIDConstants(
    double kP,
    double kI,
    double kD,
    double kS,
    double kG,
    double kV,
    double kA) {

    public static final PIDConstants ZERO = new PIDConstants(0, 0, 0, 0, 0, 0, 0);

    public PIDConstants withKP(double kP) {
        return new PIDConstants(kP, kI, kD, kS, kG, kV, kA);
    }

    public PIDConstants withKI(double kI) {
        return new PIDConstants(kP, kI, kD, kS, kG, kV, kA);
    }

    public PIDConstants withKD(double kD) {
        return new PIDConstants(kP, kI, kD, kS, kG, kV, kA);
    }

    public PIDConstants withKS(double kS) {
        return new PIDConstants(kP, kI, kD, kS, kG, kV, kA);
    }

    public PIDConstants withKG(double kG) {
        return new PIDConstants(kP, kI, kD, kS, kG, kV, kA);
    }

    public PIDConstants withKV(double kV) {
        return new PIDConstants(kP, kI, kD, kS, kG, kV, kA);
    }

    public PIDConstants withKA(double kA) {
        return new PIDConstants(kP, kI, kD, kS, kG, kV, kA);
    }
}
