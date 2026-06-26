package frc.robot.util;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class RollerMechanism2D {
    // Coarse polygon on the RIO to keep AdvantageKit serialization cheap; full detail in sim.
    private static final int ROLLER_SIDES = Constants.CURRENT_MODE == Mode.REAL ? 3 : 20;
    private static final double ROLLER_RADIUS = 0.4;

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("RollerSpinner", 0.5, 0.5);
    private final LoggedMechanismLigament2d spike;

    public RollerMechanism2D(Color8Bit rollerColor, Color8Bit spikeColor, double rollerRadius) {
        double edgeLength = 2.0 * rollerRadius * Math.sin(Math.PI / ROLLER_SIDES);
        double edgeTurnDeg = 360.0 / ROLLER_SIDES;
        double firstEdgeOffsetDeg = 90.0 + edgeTurnDeg / 2.0;

        spike = mechanismRoot.append(
            new LoggedMechanismLigament2d(
                "Spike", rollerRadius, 0.0, 6.0, spikeColor));

        LoggedMechanismLigament2d previous = spike.append(
            new LoggedMechanismLigament2d(
                "Edge0", edgeLength, firstEdgeOffsetDeg, 4.0, rollerColor));

        for (int i = 1; i < ROLLER_SIDES; i++) {
            previous = previous.append(
                new LoggedMechanismLigament2d(
                    "Edge" + i, edgeLength, edgeTurnDeg, 4.0, rollerColor));
        }
    }

    public RollerMechanism2D(double rollerRadius) {
        this(new Color8Bit(Color.kBlueViolet), new Color8Bit(Color.kOrange), rollerRadius);
    }

    public RollerMechanism2D(Color8Bit rollerColor, Color8Bit spikeColor) {
        this(rollerColor, spikeColor, ROLLER_RADIUS);
    }

    public RollerMechanism2D() {
        this(new Color8Bit(Color.kBlueViolet), new Color8Bit(Color.kOrange), ROLLER_RADIUS);
    }

    public LoggedMechanism2d getMechanism2d() {
        return mechanism;
    }

    public void setPosition(double positionRot) {
        spike.setAngle(Units.rotationsToDegrees(positionRot));
    }
}
