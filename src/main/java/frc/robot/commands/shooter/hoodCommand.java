package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.Intertable;
import frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Intertable;

public class hoodCommand extends Command{

    private hood hd;
    private double distance = 0;
    private Intertable tableThing = new Intertable();

    public hoodCommand(hood h, double b){
        this.hd = h;
        this.distance=b;
        addRequirements(hd);
    }
    @Override
    public void initialize(){
        double ang = tableThing.getAngle(distance);
        System.out.println("hood angle: " + ang );
        hd.setHoodAngle(ang);

    }
    @Override
    public void execute() {
    }
    @Override
    public boolean isFinished(){
        return true;
    }
    @Override
    public void end(boolean interrupted){
    }
}