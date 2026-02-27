package frc.robot.commands.hopper;

import frc.robot.subsystems.hopper.HopperSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class indexerRun extends Command{

    private HopperSubsystem hop;
    
    public indexerRun(HopperSubsystem h){
        hop = h;
        addRequirements(hop);
    }

    @Override
    public void execute(){
        hop.spinAtTargetRPM();
    }

    @Override
    public void end(boolean interrupted){
        hop.spinAtRPM(0);
    }
}