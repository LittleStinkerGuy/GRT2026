package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.Intertable;

import com.google.flatbuffers.Table;

import edu.wpi.first.wpilibj2.command.Command;

public class rampDownFlywheel extends Command{
    
    private flywheel fly;
    
    public rampDownFlywheel(flywheel h){
        this.fly = h;
        addRequirements(fly);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    @Override
    public void end(boolean interrupted){
        fly.dontShoot();
    }

}