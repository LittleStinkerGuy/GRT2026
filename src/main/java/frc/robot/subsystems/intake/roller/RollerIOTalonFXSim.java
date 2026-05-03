package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;

public class RollerIOTalonFXSim extends RollerIOTalonFX {
    private static final double LOOP_PERIOD_SECONDS = 0.02;

    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(IntakeConstants.ROLLER_SIM_P)
        .withKV(IntakeConstants.ROLLER_SIM_V);

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim sim;
    private final TalonFXSimState motorSimState;

    public RollerIOTalonFXSim(LoggedCanivore canivore) {
        super(canivore);
        motorSimState = motor.getSimState();

        double kARadPerSec2 = IntakeConstants.ROLLER_A / (2 * Math.PI);
        double kT = gearbox.stallTorqueNewtonMeters / gearbox.stallCurrentAmps;
        double resistance = gearbox.nominalVoltageVolts / gearbox.stallCurrentAmps;
        double moiKgM2 = (kARadPerSec2 * kT) / resistance;

        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, moiKgM2, 1.0),
            gearbox);

        motorSimState.Orientation = IntakeConstants.ROLLER_INVERTED == InvertedValue.Clockwise_Positive
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(motorSimState.getMotorVoltageMeasure().in(Volts));
        sim.update(LOOP_PERIOD_SECONDS);

        motorSimState.setRawRotorPosition(sim.getAngularPosition());
        motorSimState.setRotorVelocity(sim.getAngularVelocity());

        super.updateInputs(inputs);
    }
}
