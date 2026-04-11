package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.intake.PivotAndRollerIntakeCommand;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;

public class CNeutralIntakeAuton extends SequentialCommandGroup {
    private static final double SHOOT_TIMEOUT_SECONDS = 3.0;

    public CNeutralIntakeAuton(
        flywheel flySubsystem,
        hood hoodSubsystem,
        towerRollers towerSubsystem,
        HopperSubsystem hopperSubsystem,
        PivotIntakeSubsystem pivotIntakeSubsystem,
        RollerIntakeSubsystem rollerSubsystem) {

        PathPlannerPath toNeutralC;
        PathPlannerPath neutralIntakeC;

        try {
            toNeutralC = PathPlannerPath.fromPathFile("ToNeutralC");
            neutralIntakeC = PathPlannerPath.fromPathFile("NeutralIntakeC");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        addCommands(
            AutoBuilder.resetOdom(toNeutralC.getStartingHolonomicPose().get()),
            AutoBuilder.followPath(toNeutralC),

            Commands.parallel(
                new PivotAndRollerIntakeCommand(pivotIntakeSubsystem, rollerSubsystem),
                AutoBuilder.followPath(neutralIntakeC)),

            new AutonShooterSequence(
                flySubsystem,
                hoodSubsystem,
                towerSubsystem,
                hopperSubsystem,
                pivotIntakeSubsystem).withTimeout(SHOOT_TIMEOUT_SECONDS));
    }
}
