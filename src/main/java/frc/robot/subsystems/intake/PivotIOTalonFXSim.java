package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.PIDConstants;

public class PivotIOTalonFXSim extends PivotIOTalonFX {
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final PIDConstants DEFAULT_PID = PIDConstants.ZERO
        .withKP(IntakeConstants.PIVOT_SIM_P)
        .withKD(IntakeConstants.PIVOT_SIM_D)
        .withKG(IntakeConstants.PIVOT_SIM_G);

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    private final TalonFXSimState motorSimState;
    private final CANcoderSimState cancoderSimState;
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        gearbox,
        IntakeConstants.GEAR_RATIO,
        IntakeConstants.PIVOT_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
        IntakeConstants.PIVOT_COM_LENGTH.in(Meters),
        IntakeConstants.PIVOT_REVERSE_LIMIT.in(Radians),
        IntakeConstants.PIVOT_FORWARD_LIMIT.in(Radians),
        true,
        0.0);

    public PivotIOTalonFXSim(LoggedCanivore canivore) {
        super(canivore);

        motorSimState = motor.getSimState();
        cancoderSimState = cancoder.getSimState();

        motorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        cancoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;

        cancoder.getPosition(false).setUpdateFrequency(250);
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(motorSimState.getMotorVoltageMeasure().in(Volts));
        sim.update(LOOP_PERIOD_SECONDS);

        cancoderSimState.setRawPosition(Radians.of(sim.getAngleRads()));
        cancoderSimState.setVelocity(RadiansPerSecond.of(sim.getVelocityRadPerSec()));

        motorSimState.setRawRotorPosition(Radians.of(sim.getAngleRads()).times(IntakeConstants.GEAR_RATIO));

        super.updateInputs(inputs);
    }
}
