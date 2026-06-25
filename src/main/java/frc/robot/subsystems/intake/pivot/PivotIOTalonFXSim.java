package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
        IntakeConstants.PIVOT_MOMENT_OF_INERTIA_KG_M2,
        IntakeConstants.PIVOT_COM_LENGTH_M,
        Units.rotationsToRadians(IntakeConstants.PIVOT_REVERSE_LIMIT_ROT),
        Units.rotationsToRadians(IntakeConstants.PIVOT_FORWARD_LIMIT_ROT),
        true,
        0.0);

    public PivotIOTalonFXSim(LoggedCanivore canivore) {
        super(canivore);

        motorSimState = motor.getSimState();
        cancoderSimState = cancoder.getSimState();

        motorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        cancoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public PIDConstants getDefaultPID() {
        return DEFAULT_PID;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(motorSimState.getMotorVoltage());
        sim.update(LOOP_PERIOD_SECONDS);

        double mechanismRotations = Units.radiansToRotations(sim.getAngleRads());
        double mechanismRPS = Units.radiansToRotations(sim.getVelocityRadPerSec());

        cancoderSimState.setRawPosition(mechanismRotations);
        cancoderSimState.setVelocity(mechanismRPS);

        motorSimState.setRawRotorPosition(mechanismRotations * IntakeConstants.GEAR_RATIO);
        motorSimState.setRotorVelocity(mechanismRPS * IntakeConstants.GEAR_RATIO);

        super.updateInputs(inputs);
    }
}
