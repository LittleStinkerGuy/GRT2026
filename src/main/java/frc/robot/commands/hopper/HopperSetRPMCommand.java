package frc.robot.commands.hopper;

// --- Set RPM command (commented out for now - using manual control) ---
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.hopper.HopperMotor;
//
// public class HopperSetRPMCommand extends Command {
//     private final HopperMotor hopperMotor;
//
//     public HopperSetRPMCommand(HopperMotor subsystem) {
//         this.hopperMotor = subsystem;
//         addRequirements(subsystem);
//     }
//
//     @Override
//     public void initialize() {
//         hopperMotor.spinAtTargetRPM();
//     }
//
//     @Override
//     public void execute() {
//     }
//
//     @Override
//     public void end(boolean interrupted) {
//         hopperMotor.stop();
//     }
//
//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
