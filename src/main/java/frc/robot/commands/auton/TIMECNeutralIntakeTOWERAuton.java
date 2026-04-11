package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TowerShot;
import frc.robot.commands.intake.pivot.PivotOutCommand;
import frc.robot.commands.intake.roller.RollerInCommand;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.shooterLearner;
import frc.robot.subsystems.shooter.towerRollers;

// WITH TIME!!!!! THIS IS AUTON WHERE WE END SHOOTING @ TOWER

// keep in mind that this auton may end before the last command fully executes
// the goal is just to get as close as possible to the tower so that we can shoot asap when teleop starts

public class TIMECNeutralIntakeTOWERAuton extends SequentialCommandGroup {
    private static final double SHOOT_TIMEOUT_SECONDS = 6.0;

    public TIMECNeutralIntakeTOWERAuton(
        flywheel flySubsystem,
        hood hoodSubsystem,
        towerRollers towerSubsystem,
        HopperSubsystem hopperSubsystem,
        PivotIntakeSubsystem pivotIntakeSubsystem,
        RollerIntakeSubsystem rollerSubsystem,
        shooterLearner learnerSubsystem) {

        PathPlannerPath optimizedStartC;
        PathPlannerPath neutralIntakeC;
        PathPlannerPath towerFromNeutralC;
        PathPlannerPath fromTowerC;
        PathPlannerPath fromNeutralC;

        try {
            optimizedStartC = PathPlannerPath.fromPathFile("OPTIMIZEDSTARTC");
            neutralIntakeC = PathPlannerPath.fromPathFile("NeutralIntakeC");
            towerFromNeutralC = PathPlannerPath.fromPathFile("TOWER_FromNeutralC");
            fromTowerC = PathPlannerPath.fromPathFile("FROMTOWERC");
            fromNeutralC = PathPlannerPath.fromPathFile("FromNeutralC");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        addCommands(
            AutoBuilder.resetOdom(optimizedStartC.getStartingHolonomicPose().get()),

            // OPTIMIZEDSTARTC with timed PivotDown at approx halfway down path
            Commands.deadline(
                AutoBuilder.followPath(optimizedStartC),
                Commands.sequence(
                    Commands.waitSeconds(0.67),
                    new PivotOutCommand(pivotIntakeSubsystem))),

            // NeutralIntakeC with roller intake
            Commands.deadline(
                AutoBuilder.followPath(neutralIntakeC),
                new RollerInCommand(rollerSubsystem)),

            AutoBuilder.followPath(towerFromNeutralC),

            new TowerShot(
                flySubsystem,
                hoodSubsystem,
                towerSubsystem,
                hopperSubsystem,
                pivotIntakeSubsystem,
                learnerSubsystem).withTimeout(SHOOT_TIMEOUT_SECONDS),

            AutoBuilder.followPath(fromTowerC),

            // PivotDown before second intake
            new PivotOutCommand(pivotIntakeSubsystem).withTimeout(1.0),

            // Second NeutralIntakeC with roller intake
            Commands.deadline(
                AutoBuilder.followPath(neutralIntakeC),
                new RollerInCommand(rollerSubsystem)),

            AutoBuilder.followPath(fromNeutralC));
    }
}
