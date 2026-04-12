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

public class NeutralDefenseAuton extends SequentialCommandGroup {

    public NeutralDefenseAuton(
        flywheel flySubsystem,
        hood hoodSubsystem,
        towerRollers towerSubsystem,
        HopperSubsystem hopperSubsystem,
        PivotIntakeSubsystem pivotIntakeSubsystem,
        RollerIntakeSubsystem rollerSubsystem) {

        PathPlannerPath ToNeutralA;

        try {
            ToNeutralA = PathPlannerPath.fromPathFile("ToNeutralA");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        addCommands(
            AutoBuilder.followPath(ToNeutralA));
    }
}
