// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

//Level 2 - generated by ryan
/** An example command that uses an example subsystem. */
public class LimelightHorizontalAlign extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private boolean aligned = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimelightHorizontalAlign(DriveSubsystem m_drive){
    this.m_drive = m_drive;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("LimelightHorizontalAligh has started");
    aligned = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(LimelightHelpers.getTX("")) > 1 || LimelightHelpers.getTX("") == 0){
      if(Math.abs(LimelightConstants.HORkP * LimelightHelpers.getTX("") * -1 * DriveConstants.kMaxSpeedMetersPerSecond) > LimelightConstants.MaxAlignSpeed)
      {
        double ySpeed = LimelightConstants.MaxAlignSpeed * Math.signum(LimelightHelpers.getTX("")) * -1;
        m_drive.drive(0, ySpeed, 0, false);
      }
      else
      {
        double ySpeed = LimelightConstants.HORkP * LimelightHelpers.getTX("") * -1 * DriveConstants.kMaxSpeedMetersPerSecond;
        m_drive.drive(0, ySpeed, 0, false);
      }
    }
    else
    {
      aligned = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0,0,0,true);
    System.out.println("LimelightHorizontalAligh has ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return aligned;
  }
}
