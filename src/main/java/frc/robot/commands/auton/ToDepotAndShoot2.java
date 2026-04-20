package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.TowerShot;
import frc.robot.commands.intake.PivotAndRollerIntakeCommand;
import frc.robot.commands.intake.pivot.PivotOutCommand;
import frc.robot.commands.intake.roller.RollerInCommand;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterLearner;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;

public class ToDepotAndShoot2 extends SequentialCommandGroup {

    private static final double SHOOT_TIMEOUT_SECONDS = 3.0;
    private static final double TOWER_SHOOT_TIMEOUT_SECONDS = 20.0;

    public ToDepotAndShoot2(
        FlywheelSubsystem flySubsystem,
        HoodSubsystem hoodSubsystem,
        TowerRollersSubsystem towerSubsystem,
        HopperSubsystem hopperSubsystem,
        PivotIntakeSubsystem pivotIntakeSubsystem,
        RollerIntakeSubsystem rollerSubsystem,
        ShooterLearner learnerSubsystem) {

        PathPlannerPath path1;
        PathPlannerPath path1_5;
        PathPlannerPath path2;

        try {
            path1 = PathPlannerPath.fromPathFile("path1");
            path1_5 = PathPlannerPath.fromPathFile("path 1.5");
            path2 = PathPlannerPath.fromPathFile("path2");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        addCommands(
            // Reset odometry
            AutoBuilder.resetOdom(path1.getStartingHolonomicPose().get()),

            new AutonShooterSequence(
                flySubsystem,
                hoodSubsystem,
                towerSubsystem,
                hopperSubsystem,
                pivotIntakeSubsystem).withTimeout(SHOOT_TIMEOUT_SECONDS),

            // path1 alone (no intake yet)
            AutoBuilder.followPath(path1),

            // Run intake WITH path 1.5 (changed from parallel cmd) formerky deadline
            Commands.deadline(
                AutoBuilder.followPath(path1_5),
                new PivotOutCommand(pivotIntakeSubsystem)),


            // Run rollers @ depot
            new RollerInCommand(rollerSubsystem)
                .withTimeout(3.0),

            AutoBuilder.followPath(path2),

            new TowerShot(
                flySubsystem,
                hoodSubsystem,
                towerSubsystem,
                hopperSubsystem,
                pivotIntakeSubsystem,
                learnerSubsystem).withTimeout(TOWER_SHOOT_TIMEOUT_SECONDS));
    }
}
