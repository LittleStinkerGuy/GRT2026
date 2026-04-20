package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.allign.AimWhileDrivingCommand;
import frc.robot.commands.hopper.RunIndexer;
import frc.robot.commands.intake.roller.RollerOutCommand;
import frc.robot.commands.shooter.HoodCommand;
import frc.robot.commands.shooter.RampFlywheel;
import frc.robot.commands.shooter.towerrollers.TowerRoll;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;

public class ShooterSequence extends ParallelCommandGroup {

    public ShooterSequence(
        FlywheelSubsystem fly,
        HoodSubsystem hood,
        TowerRollersSubsystem tower,
        HopperSubsystem hopperSubsystem,
        RollerIntakeSubsystem rollerIntake) {

        // All run simultaneously:
        // - Swerve aims at target while allowing driver input
        // - Flywheel ramps up to calculated speed
        // - Hood adjusts to calculated angle
        // - Tower feeds balls only when FlywheelSubsystem is at speed
        // - Indexer feeds balls only when FlywheelSubsystem is at speed
        addCommands(
            new RampFlywheel(fly), // ramp FlywheelSubsystem
            // new RollerOutCommand(rollerIntake, -0.75), // rollers out
            new HoodCommand(hood), // set hood angle
            new TowerRoll(tower), // set tower rollers
            new RunIndexer(hopperSubsystem)); // runs after x seconds tuned in indexerRun
    }
}
