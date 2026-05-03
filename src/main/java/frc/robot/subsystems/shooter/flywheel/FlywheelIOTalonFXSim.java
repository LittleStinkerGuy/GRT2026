package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;

public class FlywheelIOTalonFXSim extends FlywheelIOTalonFX {
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    // Plant inertia for sim. Flywheel kA isn't tuned yet; pick a reasonable default for the wheel.
    private static final double FLYWHEEL_MOI_KG_M2 = 0.025;

    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(ShooterConstants.Flywheel.SIM_KP)
        .withKV(ShooterConstants.Flywheel.SIM_KV);

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(2);
    private final DCMotorSim sim;
    private final TalonFXSimState leaderSimState;

    public FlywheelIOTalonFXSim(LoggedCanivore canivore) {
        super(canivore);
        leaderSimState = leader.getSimState();

        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, FLYWHEEL_MOI_KG_M2, ShooterConstants.Flywheel.GEAR_RATIO),
            gearbox);

        leaderSimState.Orientation = ShooterConstants.Flywheel.F_INVERTED_VALUE == InvertedValue.Clockwise_Positive
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(leaderSimState.getMotorVoltageMeasure().in(Volts));
        sim.update(LOOP_PERIOD_SECONDS);

        leaderSimState.setRawRotorPosition(sim.getAngularPosition().times(ShooterConstants.Flywheel.GEAR_RATIO));
        leaderSimState.setRotorVelocity(sim.getAngularVelocity().times(ShooterConstants.Flywheel.GEAR_RATIO));

        super.updateInputs(inputs);
    }
}
