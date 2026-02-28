package frc.robot.commands;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.*;
import frc.robot.Constants.AlignToHubConstants;

import frc.robot.commands.shooter.rampFlywheel;
import frc.robot.commands.shooter.hoodCommand;
import frc.robot.commands.shooter.rampDownFlywheel;
import frc.robot.commands.hopper.indexerRun;

import frc.robot.commands.allign.AimToHubCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import com.google.flatbuffers.Table;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;

public class ShooterSequence extends SequentialCommandGroup {

    public ShooterSequence(SwerveSubsystem swerve, flywheel fly, hood hood, HopperSubsystem hopper, FieldManagementSubsystem fms) {

        double dist = swerve.getRobotPosition().getTranslation().getDistance(AlignToHubConstants.HUB_POSITION);
        AimToHubCommand aimToHubCommand = new AimToHubCommand(swerve, fms);
        Timer timer = new Timer();
        timer.start();
        System.out.println("Dist " + dist);
        
        addCommands(
            // Aim first
            new rampFlywheel(fly, dist),
            new hoodCommand(hood, dist),

            Commands.defer(() -> aimToHubCommand.createAimCommand(() -> timer.hasElapsed(4.0)), java.util.Set.of(swerve)),
            // Then shoot
            new ParallelCommandGroup( 
                new indexerRun(hopper)
            )
        );
    }
}