package frc.robot.commands;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.shooter.towerRollers;

import frc.robot.commands.shooter.rampFlywheel;
import frc.robot.commands.shooter.hoodCommand;
import frc.robot.commands.hopper.indexerRun;
import frc.robot.commands.intake.roller.RollerOutCommand;
import frc.robot.commands.shooter.towerRollers.towerRoll;
import frc.robot.commands.allign.AimWhileDrivingCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.function.DoubleSupplier;

public class ShooterSequence extends ParallelCommandGroup {

    public ShooterSequence(
        flywheel fly,
        hood hood,
        towerRollers tower,
        HopperSubsystem hopperSubsystem,
        PivotIntakeSubsystem pivotIntake) {

        // All run simultaneously:
        // - Swerve aims at target while allowing driver input
        // - Flywheel ramps up to calculated speed
        // - Hood adjusts to calculated angle
        // - Tower feeds balls only when flywheel is at speed
        // - Indexer feeds balls only when flywheel is at speed
        addCommands(
            new rampFlywheel(fly), // ramp flywheel
            new RollerOutCommand(pivotIntake), // rollers out
            new hoodCommand(hood), // set hood angle
            new towerRoll(tower), // set tower rollers
            new indexerRun(hopperSubsystem));// runs after x seconds tuned in indexerRun
    }
}
