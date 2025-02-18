// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.WristConstants;



public class WristSubsystem extends SubsystemBase {
 
  SparkMax wristMotor;

  RelativeEncoder wristEncoder; 

  ProfiledPIDController wristPIDController = new ProfiledPIDController(
    WristConstants.kP, WristConstants.kI, WristConstants.kD,
    new TrapezoidProfile.Constraints(WristConstants.MaxPIDVelocity, WristConstants.MaxPIDAcceleration));

  private double PIDgoal;
  private boolean goalSet;
  
  public WristSubsystem() {


    wristMotor = new SparkMax(WristConstants.WristMotorID, MotorType.kBrushless); 

    wristMotor.configure(Configs.Wrist.wristMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    wristEncoder = wristMotor.getEncoder(); 

    wristEncoder.setPosition(0);

    goalSet = false;
  }

  public double getPosition() {
    return wristEncoder.getPosition();
  }

  // sets the arm speed so long as isnt too high or low
  public void setWristSpeed(double speed){
    if(getPosition() < WristConstants.MaxWristMargin && getPosition() > WristConstants.MinWristMargin){
      wristMotor.set(speed);
    }
    else if(getPosition() >= WristConstants.MaxWristMargin)
    {
      if(speed < 0){
        wristMotor.set(speed);
      } 
      else 
      {
        wristMotor.stopMotor();
      }
    }
    else if(getPosition() <= WristConstants.MinWristMargin)
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

  /**
   * Example command factory method.
   *
   * @return a command
   */

  // @Override
  // public void periodic() {
  //       // only uncomment if we are sure the profiled PID can work at all times, also add a failsafe button
  //   if(getPosition() < PIDgoal - WristConstants.PIDErrorAllowed && getPosition() > PIDgoal + WristConstants.PIDErrorAllowed && goalSet){
  //     setWristSpeed(wristPIDController.calculate(getPosition()));
  //   }
  //   else
  //   {
  //     wristMotor.stopMotor();
  //   }
  // }

  public Command autoSetWristSpeed(double speed){
    return runOnce(
      () -> setWristSpeed(speed)
    );
  }

  // can be called to manually make the PID go to our desired goal
  // doesnt have a second failsafe for a goal too far
  public void manualGoToGoal(double goal){
    this.PIDgoal = goal;
    setWristSpeed(wristPIDController.calculate(getPosition(), goal));
    goalSet = true;
  }

  public Command autoManualGoToGoal(double goal){
    return runOnce(
      () -> manualGoToGoal(goal)
    );
  }

  // sets a goal and for as long as the robot is on
  // the arm will do its best to reach this goal and is used through periodic
  // public void setGoal(double goal){
  //   this.PIDgoal = goal;
  //   if(goal < WristConstants.MaxWristMargin && goal > WristConstants.MinWristMargin){
  //     wristPIDController.setGoal(goal);
  //     goalSet = true;
  //   }
  // }

  // public Command setGoalCmd(double goal){
  //   return runOnce(() -> { 
  //     this.PIDgoal = goal; 
  //   });
  // }

}
