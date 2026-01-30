package frc.robot.subsystems.shooter;

import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.EnumSet;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus;

public class railgun extends SubsystemBase {

    private CANBus canivore;
    private TalonFX upperMotor = new TalonFX(railgunConstants.upperId, canivore);
    private TalonFX hoodMotor = new TalonFX(railgunConstants.hoodId, canivore);
    private CANdi limit = new CANdi(railgunConstants.limitId, "can");

    double hoodAngle = railgunConstants.initHoodAngle;

    public railgun(CANBus c) {
        hoodMotor.setPosition(railgunConstants.initHoodAngle);
        canivore = c;
    }

    public void input(double r, boolean l, int arrow, boolean options) { // check logic here again

        // hood
        if (arrow == 180 /*
                          * && hoodMotor.getPosition().getValueAsDouble() - 0.014 >=
                          * railgunConstants.lowerAngle
                          */) {

            hoodMotor.set(0.1);

        } else if (arrow == 0 /*
                               * && hoodMotor.getPosition().getValueAsDouble() + 0.014 <=
                               * railgunConstants.upperAngle
                               */) {

            hoodMotor.set(-0.1);
        }

        SmartDashboard.putNumber("help", hoodMotor.getAcceleration().getValueAsDouble());

        upperMotor.set((r + 1) / 2);

    }

    public void periodic() {

    }

}
