package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.intake.roller.RollerInCommand;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;

// This auton shoots preloaded balls, drives to depot & intakes balls, drives back to hub to shoot.
public class ToDepotAndShoot extends SequentialCommandGroup {

    private static final double SHOOT_TIMEOUT_SECONDS = 3.0;

    public ToDepotAndShoot(
        flywheel flySubsystem,
        hood hoodSubsystem,
        towerRollers towerSubsystem,
        HopperSubsystem hopperSubsystem,
        PivotIntakeSubsystem pivotIntakeSubsystem,
        RollerIntakeSubsystem rollerSubsystem) {

        PathPlannerPath path1;
        PathPlannerPath path2;

        try {
            path1 = PathPlannerPath.fromPathFile("path1");
            path2 = PathPlannerPath.fromPathFile("path2");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        addCommands(
            new AutonShooterSequence(
                flySubsystem,
                hoodSubsystem,
                towerSubsystem,
                hopperSubsystem,
                pivotIntakeSubsystem).withTimeout(SHOOT_TIMEOUT_SECONDS),

            AutoBuilder.followPath(path1),

            new RollerInCommand(rollerSubsystem),

            AutoBuilder.followPath(path2),

            new AutonShooterSequence(
                flySubsystem,
                hoodSubsystem,
                towerSubsystem,
                hopperSubsystem,
                pivotIntakeSubsystem).withTimeout(SHOOT_TIMEOUT_SECONDS));
    }
}
