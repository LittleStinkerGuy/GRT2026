package frc.robot.commands;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.shooter.towerRollers;

import frc.robot.commands.shooter.flywheelAutoShoot;
import frc.robot.commands.shooter.hoodAuto;
import frc.robot.commands.hopper.indexerRun;
import frc.robot.commands.shooter.towerRollers.towerRoll;
import frc.robot.commands.allign.AimWhileDrivingCommand;
import frc.robot.commands.intake.pivot.PivotToggleCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.function.DoubleSupplier;

public class ShooterSeqAutoShoot extends SequentialCommandGroup {

    public ShooterSeqAutoShoot(
        SwerveSubsystem swerve,
        flywheel fly,
        hood hood,
        HopperSubsystem hopper,
        PivotIntakeSubsystem pivotIntake,
        FieldManagementSubsystem fms,
        towerRollers b,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed) {

        boolean redTeam = fms.isRedAlliance();

        AimWhileDrivingCommand aimWhileDriving =
            new AimWhileDrivingCommand(
                swerve,
                fms,
                xSpeed,
                ySpeed);

        addCommands(

            new ParallelCommandGroup(
                aimWhileDriving,
                new PivotToggleCommand(pivotIntake),
                new flywheelAutoShoot(fly, redTeam),
                new hoodAuto(hood, redTeam),
                new towerRoll(b),

                new indexerRun(hopper)));
    }
}
