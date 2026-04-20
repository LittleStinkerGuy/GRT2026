package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.TowerShot;
import frc.robot.commands.intake.pivot.PivotOutCommand;
import frc.robot.commands.intake.roller.RollerInCommand;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterLearner;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;

public class IsolatedAuto extends SequentialCommandGroup {
    private static final double SHOOT_TIMEOUT_SECONDS = 3.0;

    public IsolatedAuto(
        FlywheelSubsystem flySubsystem,
        HoodSubsystem hoodSubsystem,
        TowerRollersSubsystem towerSubsystem,
        HopperSubsystem hopperSubsystem,
        PivotIntakeSubsystem pivotIntakeSubsystem,
        RollerIntakeSubsystem rollerSubsystem,
        ShooterLearner learnerSubsystem) {

        PathPlannerPath toNeutralC;
        PathPlannerPath neutralIntakeC;
        PathPlannerPath fromNeutralC;

        try {
            toNeutralC = PathPlannerPath.fromPathFile("ToNeutralC");
            neutralIntakeC = PathPlannerPath.fromPathFile("NeutralIntakeC");
            fromNeutralC = PathPlannerPath.fromPathFile("TOWER_FromNeutralC");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        addCommands(
            Commands.deadline(
                AutoBuilder.followPath(neutralIntakeC),
                new RollerInCommand(rollerSubsystem)));
    }
}
