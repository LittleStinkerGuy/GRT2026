package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;

public class HoodIOTalonFXSim extends HoodIOTalonFX {
    private static final double LOOP_PERIOD_SECONDS = 0.02;

    // Plant inertia / arm length placeholders — tune once Hood CAD numbers exist.
    private static final MomentOfInertia HOOD_MOI = KilogramSquareMeters.of(0.01);
    private static final Distance HOOD_COM_LENGTH = Meters.of(0.1);

    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(60.0)
        .withKD(2.0);

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    private final TalonFXSimState motorSimState;
    private final CANcoderSimState cancoderSimState;
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        gearbox,
        ShooterConstants.Hood.GEAR_RATIO,
        HOOD_MOI.in(KilogramSquareMeters),
        HOOD_COM_LENGTH.in(Meters),
        ShooterConstants.Hood.LOWER_ANGLE_LIMIT.in(Radians),
        ShooterConstants.Hood.UPPER_ANGLE_LIMIT.in(Radians),
        false,
        ShooterConstants.Hood.INIT_ANGLE.in(Radians));

    public HoodIOTalonFXSim(LoggedCanivore canivore) {
        super(canivore);

        motorSimState = motor.getSimState();
        cancoderSimState = cancoder.getSimState();

        // Match the sign-flip baked into the real motor's feedback config (negative
        // RotorToSensorRatio) so simulated rotor motion produces the expected
        // mechanism-direction position.
        motorSimState.Orientation = ChassisReference.Clockwise_Positive;
        cancoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(motorSimState.getMotorVoltageMeasure().in(Volts));
        sim.update(LOOP_PERIOD_SECONDS);

        cancoderSimState.setRawPosition(Radians.of(sim.getAngleRads()));
        cancoderSimState.setVelocity(RadiansPerSecond.of(sim.getVelocityRadPerSec()));

        motorSimState.setRawRotorPosition(Radians.of(sim.getAngleRads() * ShooterConstants.Hood.GEAR_RATIO));
        motorSimState.setRotorVelocity(RadiansPerSecond.of(sim.getVelocityRadPerSec() * ShooterConstants.Hood.GEAR_RATIO));

        super.updateInputs(inputs);
    }
}
