package frc.robot.util;

import static edu.wpi.first.units.Units.NewtonMeters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class LoggedTalon extends TalonFX {
    enum TelemetryLevel {
        BASIC,
        STANDARD,
        DETAILED,
        FULL;

        boolean includes(TelemetryLevel required) {
            return this.ordinal() >= required.ordinal();
        }
    }

    private static final TelemetryLevel DEFAULT_TELEMETRY_LEVEL = TelemetryLevel.BASIC;
    private final String logPrefix;

    public TelemetryLevel dashboardTelemetryLevel = DEFAULT_TELEMETRY_LEVEL;
    public TelemetryLevel logTelemetryLevel = DEFAULT_TELEMETRY_LEVEL;

    // ===== Cached signals (refresh once, then read many) =====
    private final StatusSignal<Angle> position = getPosition();
    private final StatusSignal<AngularVelocity> velocity = getVelocity();
    private final StatusSignal<AngularAcceleration> acceleration = getAcceleration();

    private final StatusSignal<Double> dutyCycle = getDutyCycle();
    private final StatusSignal<Voltage> supplyVoltage = getSupplyVoltage();
    private final StatusSignal<Voltage> appliedVoltage = getMotorVoltage();
    private final StatusSignal<Current> supplyCurrent = getSupplyCurrent();
    private final StatusSignal<Current> statorCurrent = getStatorCurrent();

    private final StatusSignal<Temperature> deviceTemp = getDeviceTemp();

    private final StatusSignal<Current> torqueCurrent = getTorqueCurrent();
    private final StatusSignal<Per<TorqueUnit, CurrentUnit>> motorKt = getMotorKT();

    private final StatusSignal<Double> closedLoopReference = getClosedLoopReference();
    private final StatusSignal<Double> closedLoopError = getClosedLoopError();

    private final StatusSignal<ControlModeValue> controlMode = getControlMode();

    private final StatusSignal<ForwardLimitValue> forwardLimit = getForwardLimit();
    private final StatusSignal<ReverseLimitValue> reverseLimit = getReverseLimit();

    private final StatusSignal<Boolean> faultBridgeBrownout = getFault_BridgeBrownout();
    private final StatusSignal<Boolean> faultHardware = getFault_Hardware();
    private final StatusSignal<Boolean> faultBootDuringEnable = getFault_BootDuringEnable();


    //if no name set set to motor[deviceID]
    public LoggedTalon(int deviceId, CANBus canBus) {
        this(deviceId, canBus, ("motor" + deviceId));
    }

    public LoggedTalon(int deviceId, CANBus canBus, String dashboardKey) {
        super(deviceId, canBus);
        //the replace methods are used in case the dashboardKey gives funky results
        this.logPrefix = "TalonFX/" + dashboardKey.replace("/", "_").replace(" ", "_");
    }

    /**
     * Refresh all cached signals from CAN. This is invoked automatically whenever
     * a dashboard or log update is performed.
     *
     * @return Status of the CAN refresh transaction.
     */
    private StatusCode refreshSignals() {
        return BaseStatusSignal.refreshAll(
                position,
                velocity,
                acceleration,
                dutyCycle,
                supplyVoltage,
                appliedVoltage,
                supplyCurrent,
                statorCurrent,
                deviceTemp,
                torqueCurrent,
                motorKt,
                closedLoopReference,
                closedLoopError,
                controlMode,
                forwardLimit,
                reverseLimit,
                faultBridgeBrownout,
                faultHardware,
                faultBootDuringEnable);
    }

    /**
     * Publish the latest signal values to AdvantageKit (rate limited by
     * configuration). These values can be sent to network tables and WPILOG
     * simultaneously by configuring data receivers in Robot.
     */
    public void updateDashboard() {
        refreshSignals();

        TelemetryLevel telemetryLevel = getEffectiveTelemetryLevel();
        if (telemetryLevel.includes(TelemetryLevel.BASIC)) {
            
            Logger.recordOutput(logPrefix + "/Position", position.getValue().in(Units.Radians));
            Logger.recordOutput(logPrefix + "/Velocity", velocity.getValue().in(Units.RadiansPerSecond));
            Logger.recordOutput(logPrefix + "/DutyCycle", dutyCycle.getValue());
            Logger.recordOutput(logPrefix + "/AppliedVoltage", appliedVoltage.getValue().in(Units.Volts));
            Logger.recordOutput(logPrefix + "/ControlMode", (double) controlMode.getValue().ordinal());

            Logger.recordOutput(logPrefix + "/ForwardLimit", isForwardLimitClosed());
            Logger.recordOutput(logPrefix + "/ReverseLimit", isReverseLimitClosed());

            Logger.recordOutput(logPrefix + "/Fault/Brownout", faultBridgeBrownout.getValue());
            Logger.recordOutput(logPrefix + "/Fault/Hardware", faultHardware.getValue());
            Logger.recordOutput(logPrefix + "/Fault/BootDuringEnable", faultBootDuringEnable.getValue());
        }

        if (telemetryLevel.includes(TelemetryLevel.STANDARD)) {
            Logger.recordOutput(
                    logPrefix + "/Acceleration", acceleration.getValue().in(Units.RadiansPerSecondPerSecond));
            Logger.recordOutput(logPrefix + "/SupplyVoltage", supplyVoltage.getValue().in(Units.Volts));
            Logger.recordOutput(logPrefix + "/SupplyCurrent", supplyCurrent.getValue().in(Units.Amps));
        }

        if (telemetryLevel.includes(TelemetryLevel.DETAILED)) {
            Logger.recordOutput(logPrefix + "/StatorCurrent", statorCurrent.getValue().in(Units.Amps));
            Logger.recordOutput(logPrefix + "/TempC", deviceTemp.getValue().in(Units.Celsius));
        }

        if (telemetryLevel.includes(TelemetryLevel.FULL)) {
            Logger.recordOutput(logPrefix + "/TorqueCurrent", torqueCurrent.getValue().in(Units.Amps));
            Logger.recordOutput(logPrefix + "/TorqueNm", getTorque().in(NewtonMeters));
            Logger.recordOutput(logPrefix + "/MotorKtNmPerAmp", getMotorKtNmPerAmp());
            Logger.recordOutput(logPrefix + "/ClosedLoopRef", closedLoopReference.getValue());
            Logger.recordOutput(logPrefix + "/ClosedLoopError", closedLoopError.getValue());
        }
    }

    private TelemetryLevel getEffectiveTelemetryLevel() {
        return dashboardTelemetryLevel.ordinal() >= logTelemetryLevel.ordinal()
                ? dashboardTelemetryLevel
                : logTelemetryLevel;
    }

    private boolean isForwardLimitClosed() {
        return forwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
    }

    private boolean isReverseLimitClosed() {
        return reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
    }

    //torque = kt * torqueCurrent
    private Torque getTorque() {
        //kt = torque constant
        Per<TorqueUnit, CurrentUnit> kt = motorKt.getValue();
        Current tCurrent = torqueCurrent.getValue();
        return (Torque) kt.timesDivisor(tCurrent);
    }


    //torque = kt * torqueCurrent

    private double getMotorKtNmPerAmp() {
        return motorKt.getValue().timesDivisor(Units.Amps.of(1.0)).in(NewtonMeters);
    }
}