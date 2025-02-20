// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveClickToAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  
  private double clickedAngle = 0.0;

  //stores whether or not we are aligned with our angle
  private boolean aligned;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveClickToAngle(DriveSubsystem m_drive) {
    this.m_drive = m_drive;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveClickToAngle has started");
    clickedAngle = getClickedAngle();
    SmartDashboard.putNumber("Clicked Angle", getClickedAngle());
    aligned = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double gyroValue = m_drive.getGyroDoubleValueBounded();
    //if our clicked angle is 0, and the gyro reads 359 or something, it needs to rotate properly
    if(clickedAngle == 0 && (Math.abs(gyroValue - 360) > 2 || Math.abs(gyroValue) > 2 ) )
    {
      if(gyroValue > 270 && Math.abs(gyroValue - 360) > 2 )
      {
        m_drive.drive(0,0, 1 * DriveConstants.SlowRotSpeed, true);
      }
      else if(gyroValue < 90 && Math.abs(gyroValue - 0) > 2 )
      {
        System.out.println("second clause is being run");
        m_drive.drive(0,0, -1 * DriveConstants.SlowRotSpeed, true);
      }
      else
      {
        aligned = true;
      }
    }
    else if(gyroValue > clickedAngle && Math.abs(gyroValue - clickedAngle) > 2 )
    {
      m_drive.drive(0,0, -1 * DriveConstants.SlowRotSpeed, true);
    }
    else if(gyroValue < clickedAngle && Math.abs(gyroValue - clickedAngle) > 2 )
    {
      m_drive.drive(0,0, 1 * DriveConstants.SlowRotSpeed, true);
    }
    else 
    {
        aligned = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveClickToAngle has ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return aligned;
  }

  //for each angle increment of 60 (hexagon angles) if the robot is closest to one of the increments, set that as the angle
  public double getClickedAngle(){
    for(double i = 0; i < 360; i += 60){
        if(within30(m_drive.getGyroDoubleValueBounded(), i)){
            return i;
        }
    }
    System.out.println("Could not find angle to click to");
    return 0;
  }

  //gets if a number is within 30 of another number, since then we know where to click to
  public boolean within30(double input, double base){
    double high = base + 30;
    double low = base - 30;

    if(low < input && input < high){
        return true;
    } else {
        return false;
    }
  }
}
