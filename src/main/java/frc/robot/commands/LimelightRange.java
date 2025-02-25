// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
//LEVEL ONE - AIM AND RANGE
/** An example command that uses an example subsystem. */
public class LimelightRange extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private boolean aligned = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimelightRange(DriveSubsystem m_drive) {
    this.m_drive = m_drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Limelight Aim has Started!");
    aligned = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double area = LimelightHelpers.getTA("");
    // double rotSpeed = LimelightConstants.ROTkP * LimelightHelpers.getTX("limelight") * -1 * DriveConstants.kMaxAngularSpeed;
    if(area < LimelightConstants.CloseEnoughArea && area != 0){
      if(LimelightConstants.RANGEkP * (4.45 - area) * DriveConstants.MaxMoveInRangeSpeed >= DriveConstants.MaxMoveInRangeSpeed){
        double xSpeed = DriveConstants.MaxMoveInRangeSpeed;
        m_drive.drive(xSpeed, 0, 0, false);
      }else{
        double xSpeed = LimelightConstants.RANGEkP * (4.45 - area) * DriveConstants.MaxMoveInRangeSpeed;
        m_drive.drive(xSpeed, 0, 0, false);
      }
    } else {
      aligned = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0,0,0, true);
    System.out.println("Limelight Aim has Ended!");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return aligned;
  }
}
