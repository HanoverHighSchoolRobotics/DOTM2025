// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualMoveVerticalDistance extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private String direction;
  private double distance;
  
  private double xMultiplier; 
  private Pose2d startingPose;

  private boolean reachedGoal;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualMoveVerticalDistance(DriveSubsystem m_drive, String direction, double distance) {
    this.m_drive = m_drive;
    this.direction = direction;
    this.distance = distance;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ManualMoveVerticalDistance has started");
    m_drive.setHorizontal();
    reachedGoal = false;
    startingPose = m_drive.getPose();
    if(direction == "Down")
    {
      xMultiplier = -1;
    }
    else if(direction == "Up")
    {
      xMultiplier = 1;
    }
    else
    {
      System.out.println("In ManualMoveVerticalDistance, argument parameter input for direction is not allowed");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.pow(m_drive.getPose().getX() - startingPose.getX(), 2)
      + Math.pow(m_drive.getPose().getY() - startingPose.getY(), 2)
      < Math.pow(distance, 2))
    {
      if(Math.abs(-1 * DriveConstants.VerticalAlignmentSpeed * getPIDError()) >= DriveConstants.VerticalAlignmentSpeed)
      {
        m_drive.drive(xMultiplier * DriveConstants.VerticalAlignmentSpeed, 0, 0, false);
      } 
      else 
      {
        m_drive.drive(xMultiplier * DriveConstants.VerticalAlignmentSpeed, 0, 0, false);
      }
    }
    else
    {
      reachedGoal = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_drive.drive(0,0,0,true);
    System.out.println("ManualMoveVerticalDistance has ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reachedGoal;
  }

  public double getPIDError(){
    return (2 - (Math.sqrt(Math.pow(m_drive.getPose().getX() - startingPose.getX(), 2)
    + Math.pow(m_drive.getPose().getY() - startingPose.getY(), 2)))) * DriveConstants.HorizontalAlignmentkP;
  }
}
