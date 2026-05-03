package frc.robot.subsystems.shooter.tower;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TowerConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;

public class TowerIOTalonFXSim extends TowerIOTalonFX {
    private static final double LOOP_PERIOD_SECONDS = 0.02;

    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(TowerConstants.SIM_P)
        .withKV(TowerConstants.SIM_V);

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim sim;
    private final TalonFXSimState motorSimState;

    public TowerIOTalonFXSim(LoggedCanivore canivore) {
        super(canivore);
        motorSimState = motor.getSimState();

        double kARadPerSec2 = TowerConstants.kA / (2 * Math.PI);
        double kT = gearbox.stallTorqueNewtonMeters / gearbox.stallCurrentAmps;
        double resistance = gearbox.nominalVoltageVolts / gearbox.stallCurrentAmps;
        double moiKgM2 = (kARadPerSec2 * TowerConstants.GEAR_REDUCTION * kT) / resistance;

        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, moiKgM2, TowerConstants.GEAR_REDUCTION),
            gearbox);

        motorSimState.Orientation = TowerConstants.HOPPER_INVERTED == InvertedValue.Clockwise_Positive
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(TowerIOInputs inputs) {
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(motorSimState.getMotorVoltageMeasure().in(Volts));
        sim.update(LOOP_PERIOD_SECONDS);

        motorSimState.setRawRotorPosition(sim.getAngularPosition().times(TowerConstants.GEAR_REDUCTION));
        motorSimState.setRotorVelocity(sim.getAngularVelocity().times(TowerConstants.GEAR_REDUCTION));

        super.updateInputs(inputs);
    }
}
