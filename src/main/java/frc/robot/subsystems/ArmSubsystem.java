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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  SparkMax armMotor;

  RelativeEncoder armEncoder;

  ProfiledPIDController armPIDController = new ProfiledPIDController(
    ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
    new TrapezoidProfile.Constraints(ArmConstants.MaxPIDVelocity, ArmConstants.MaxPIDAcceleration));

  private double PIDgoal;
  private boolean goalSet;

  public ArmSubsystem() {
    armMotor = new SparkMax(ArmConstants.ArmMotorID, MotorType.kBrushless);

    armMotor.configure(Configs.Arm.armMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    armEncoder = armMotor.getEncoder();
    
    armEncoder.setPosition(0);

    goalSet = false;

    armPIDController.setTolerance(ArmConstants.PIDErrorAllowed);
  }

  @Override
  public void periodic() {

    if(goalSet && !armPIDController.atSetpoint()){
      SmartDashboard.putBoolean("Trying to go to goal", true);
      setArmSpeed(MathUtil.clamp(armPIDController.calculate(getPosition(), PIDgoal), -.75, .75));
    }
    else if (goalSet)
    {
      SmartDashboard.putBoolean("Trying to go to goal", false);
      armMotor.stopMotor();
    }
    //test to make it so when the pid gets enabled, it doesnt go to last setpoint
    else
    {
      PIDgoal = getPosition();
    }

    SmartDashboard.putNumber("Arm Encoder Reading", getPosition());
    SmartDashboard.putNumber("Calculated Arm Speed", armPIDController.calculate(getPosition(), PIDgoal));
    SmartDashboard.putNumber("Arm Goal", PIDgoal);
    SmartDashboard.putString("Arm PID Goal", armPIDController.getGoal().toString());
  }

  // sets the arm speed so long as isnt too high or low
  public void setArmSpeed(double speed){
    if(getPosition() < ArmConstants.MaxArmMargin && getPosition() > ArmConstants.MinArmMargin){
      armMotor.set(speed);
      SmartDashboard.putBoolean("Within Range", true);
    }
    else if(getPosition() >= ArmConstants.MaxArmMargin)
    {
      if(speed < 0){
        armMotor.set(speed);
      } 
      else 
      {
        armMotor.stopMotor();
      }
      SmartDashboard.putBoolean("Within Range", false);
    }
    else if(getPosition() <= ArmConstants.MinArmMargin)
    {
      if(speed > 0){
        armMotor.set(speed);
      } 
      else 
      {
        armMotor.stopMotor();
      }
      SmartDashboard.putBoolean("Within Range", false);
    }
    else
    {
      armMotor.stopMotor();
      SmartDashboard.putBoolean("Within Range", false);
    }
  }

  public Command autoSetArmSpeed(double speed){
    return runOnce(
      () -> setArmSpeed(speed)
    );
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
      armPIDController.setGoal(goal);
      goalSet = true;
    }
  }

  public Command setGoalCmd(double goal){
    return runOnce(
      () -> setGoal(goal)
    );
  }

  public void disablePID(){
    this.goalSet = false;
  }

  public Command disablePIDCmd(){
    return runOnce(
      () -> disablePID()
    );
  }
}