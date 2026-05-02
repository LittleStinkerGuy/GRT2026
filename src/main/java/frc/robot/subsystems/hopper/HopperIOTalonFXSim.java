package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.HopperConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;

public class HopperIOTalonFXSim extends HopperIOTalonFX {
    private static final double LOOP_PERIOD_SECONDS = 0.02;

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim sim;
    private final TalonFXSimState motorSimState;

    public HopperIOTalonFXSim(LoggedCanivore canivore) {
        super(canivore);
        motorSimState = motor.getSimState();

        double kARadPerSec2 = HopperConstants.kA / (2 * Math.PI);
        double kT = gearbox.stallTorqueNewtonMeters / gearbox.stallCurrentAmps;
        double resistance = gearbox.nominalVoltageVolts / gearbox.stallCurrentAmps;
        double moiKgM2 = (kARadPerSec2 * HopperConstants.GEAR_REDUCTION * kT) / resistance;

        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, moiKgM2, HopperConstants.GEAR_REDUCTION),
            gearbox);

        motorSimState.Orientation = HopperConstants.HOPPER_INVERTED == InvertedValue.Clockwise_Positive
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public PIDConstants getDefaultPID() {
        return PIDConstants.ZERO
            .withKP(HopperConstants.SIM_KP)
            .withKV(HopperConstants.SIM_KV);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(motorSimState.getMotorVoltageMeasure().in(Volts));
        sim.update(LOOP_PERIOD_SECONDS);

        motorSimState.setRawRotorPosition(sim.getAngularPosition().times(HopperConstants.GEAR_REDUCTION));
        motorSimState.setRotorVelocity(sim.getAngularVelocity().times(HopperConstants.GEAR_REDUCTION));

        super.updateInputs(inputs);
    }
}
