// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.SecondaryWristConstants;

public class SecondaryWristSubsystem extends SubsystemBase {

  SparkMax wristMotor;

  RelativeEncoder wristEncoder;

  PIDController pid = new PIDController(SecondaryWristConstants.kP, SecondaryWristConstants.kI, SecondaryWristConstants.kD);

  private double PIDgoal;
  //stores if the motor should try to go to the goal
  private boolean goalSet;

  private double desiredSpeed;

  public SecondaryWristSubsystem() {
    wristMotor = new SparkMax(SecondaryWristConstants.WristMotorID, MotorType.kBrushless);

    wristMotor.configure(Configs.SecondaryWrist.wristMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    wristEncoder = wristMotor.getEncoder();
    
    wristEncoder.setPosition(0);

    goalSet = false;

  }

  @Override
  public void periodic() {

    if(goalSet){
      wristMotor.set(MathUtil.clamp(pid.calculate(getPosition(), PIDgoal), -1 * SecondaryWristConstants.MaxPIDSetSpeed, SecondaryWristConstants.MaxPIDSetSpeed));
    }

    SmartDashboard.putNumber("Desired Wrist Speed", desiredSpeed);

    SmartDashboard.putNumber("Wrist Encoder Reading", getPosition());

    SmartDashboard.putNumber("Wrist Goal", PIDgoal);

  }

  // sets the wrist speed so long as isnt too high or low
  public void setWristSpeed(double speed){
    double position = getPosition();

    if(position < SecondaryWristConstants.MaxWristMargin && position > SecondaryWristConstants.MinWristMargin){
      wristMotor.set(speed);
    }
    else if(position >= SecondaryWristConstants.MaxWristMargin)
    {
      if(speed < 0){
        wristMotor.set(speed);
      } 
      else 
      {
        wristMotor.stopMotor();
      }
    }
    else if(position <= SecondaryWristConstants.MinWristMargin)
    {
      if(speed > 0){
        wristMotor.set(speed);
      } 
      else 
      {
        wristMotor.stopMotor();
      }
    }
    else
    {
      wristMotor.stopMotor();
    }
  }

  // only use for testing or resetting the robot position
  public Command setWristSpeedNoLimitsCmd(double speed){
    return run(
      () -> wristMotor.set(speed)
    );
  }

  public void setWristSpeedVoidCmd(double speed){
    if(speed == 0){
      goalSet = true;
    }
    else
    {
      goalSet = false;
      setWristSpeed(speed);
      PIDgoal = getPosition();
    }
  }

  // gets the wrist position in ticks
  public double getPosition(){
    return wristEncoder.getPosition();
  }

  // sets a goal and for as long as the robot is on
  // the wrist will do its best to reach this goal and is used through periodic
  public void setGoal(double goal){
    if(goal <= SecondaryWristConstants.MaxWristMargin && goal >= SecondaryWristConstants.MinWristMargin){
      this.PIDgoal = goal;
      goalSet = true;
    }
  }

  public Command setGoalCmd(double goal){
    return runOnce(
      () -> setGoal(goal)
    );
  }
}

