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
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  SparkMax armMotor;

  RelativeEncoder armEncoder;

  PIDController pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

  private double PIDgoal;
  //stores if the motor should try to go to the goal
  private boolean goalSet;

  private double desiredSpeed;

  public ArmSubsystem() {
    armMotor = new SparkMax(ArmConstants.ArmMotorID, MotorType.kBrushless);

    armMotor.configure(Configs.Arm.armMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    armEncoder = armMotor.getEncoder();
    
    armEncoder.setPosition(0);

    goalSet = false;
  }

  @Override
  public void periodic() {

    if(goalSet){
      armMotor.set(MathUtil.clamp(pid.calculate(getPosition(), PIDgoal), -1 * ArmConstants.MaxPIDSetSpeed, ArmConstants.MaxPIDSetSpeed));
    }

    SmartDashboard.putNumber("Desired Arm Speed", desiredSpeed);

    SmartDashboard.putNumber("Arm Encoder Reading", getPosition());

    SmartDashboard.putNumber("Arm Goal", PIDgoal);

  }

  // sets the arm speed so long as isnt too high or low
  public void setArmSpeed(double speed){
    double position = getPosition();

    if(position < ArmConstants.MaxArmMargin && position > ArmConstants.MinArmMargin){
      armMotor.set(speed);
    }
    else if(position >= ArmConstants.MaxArmMargin)
    {
      if(speed < 0){
        armMotor.set(speed);
      } 
      else 
      {
        armMotor.stopMotor();
      }
    }
    else if(position <= ArmConstants.MinArmMargin)
    {
      if(speed > 0){
        armMotor.set(speed);
      } 
      else 
      {
        armMotor.stopMotor();
      }
    }
    else
    {
      armMotor.stopMotor();
    }
  }

  // only use for testing or resetting the robot position
  public Command setArmSpeedNoLimitsCmd(double speed){
    return run(
      () -> armMotor.set(speed)
    );
  }

  // call using a lambda expression
  public void setArmSpeedVoidCmd(double speed){
    if(speed == 0){
      goalSet = true;
    }
    else
    {
      goalSet = false;
      setArmSpeed(speed);
      PIDgoal = getPosition();
    }
  }

  // gets the arm position in ticks
  public double getPosition(){
    return armEncoder.getPosition();
  }

  // sets a goal and for as long as the robot is on
  // the arm will do its best to reach this goal and is used through periodic
  public void setGoal(double goal){
    if(goal <= ArmConstants.MaxArmMargin && goal >= ArmConstants.MinArmMargin){
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