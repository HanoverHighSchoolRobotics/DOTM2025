package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

public class ResetGyro extends Command {

    DriveSubsystem driveSub;
    
    public ResetGyro(DriveSubsystem driveSub){
        this.driveSub = driveSub;

        addRequirements(driveSub);
    }

    public void initialize(){
        driveSub.zeroHeading();
    }

    public void execute(){}

    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}