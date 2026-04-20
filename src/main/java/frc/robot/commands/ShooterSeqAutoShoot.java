package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.allign.AimWhileDrivingCommand;
import frc.robot.commands.hopper.RunIndexer;
import frc.robot.commands.intake.pivot.PivotToggleCommand;
import frc.robot.commands.shooter.FlywheelAutoShoot;
import frc.robot.commands.shooter.HoodAuto;
import frc.robot.commands.shooter.towerrollers.TowerRoll;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterSeqAutoShoot extends SequentialCommandGroup {

    public ShooterSeqAutoShoot(
        SwerveSubsystem swerve,
        FlywheelSubsystem fly,
        HoodSubsystem hood,
        HopperSubsystem hopper,
        PivotIntakeSubsystem pivotIntake,
        FieldManagementSubsystem fms,
        TowerRollersSubsystem b,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed) {

        boolean redTeam = fms.isRedAlliance();

        AimWhileDrivingCommand aimWhileDriving =
            new AimWhileDrivingCommand(
                swerve,
                fms,
                xSpeed,
                ySpeed);

        addCommands(

            new ParallelCommandGroup(
                aimWhileDriving,
                new PivotToggleCommand(pivotIntake),
                new FlywheelAutoShoot(fly, redTeam),
                new HoodAuto(hood, redTeam),
                new TowerRoll(b),

                new RunIndexer(hopper)));
    }
}
