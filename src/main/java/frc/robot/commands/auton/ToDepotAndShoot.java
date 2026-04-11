package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.intake.PivotAndRollerIntakeCommand;
import frc.robot.commands.intake.roller.RollerInCommand;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;
import frc.robot.commands.TowerShot;
import frc.robot.subsystems.shooter.shooterLearner;

// This auton shoots preloaded balls, drives to depot & intakes balls, drives back to hub to shoot.
public class ToDepotAndShoot extends SequentialCommandGroup {

    private static final double SHOOT_TIMEOUT_SECONDS = 3.0;
    private static final double TOWER_SHOOT_TIMEOUT_SECONDS = 20.0;

    public ToDepotAndShoot(
        flywheel flySubsystem,
        hood hoodSubsystem,
        towerRollers towerSubsystem,
        HopperSubsystem hopperSubsystem,
        PivotIntakeSubsystem pivotIntakeSubsystem,
        RollerIntakeSubsystem rollerSubsystem,
        shooterLearner learnerSubsystem) {

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

            // Run intake WITH path 1.5 (changed from deadline cmd)
            Commands.parallel(
                AutoBuilder.followPath(path1_5),
                new PivotAndRollerIntakeCommand(pivotIntakeSubsystem, rollerSubsystem)),


            // Run rollers for 3 seconds @ depot
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
