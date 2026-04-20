package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.intake.PivotAndRollerIntakeCommand;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;

public class NeutralDefenseAuton extends SequentialCommandGroup {

    public NeutralDefenseAuton(
        FlywheelSubsystem flySubsystem,
        HoodSubsystem hoodSubsystem,
        TowerRollersSubsystem towerSubsystem,
        HopperSubsystem hopperSubsystem,
        PivotIntakeSubsystem pivotIntakeSubsystem,
        RollerIntakeSubsystem rollerSubsystem) {

        PathPlannerPath toNeutralA;

        try {
            toNeutralA = PathPlannerPath.fromPathFile("toNeutralA");
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        addCommands(
            AutoBuilder.followPath(toNeutralA));
    }
}
