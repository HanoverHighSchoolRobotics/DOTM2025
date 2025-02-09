// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Configs.Arm;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
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
  }

  @Override
  public void periodic() {
    // only uncomment if we are sure the profiled PID can work at all times, also add a failsafe button
    // if(getArmPos() < PIDgoal - ArmConstants.PIDErrorAllowed && getArmPos() > PIDgoal + ArmConstants.PIDErrorAllowed && goalSet){
    //   setArmSpeed(armPIDController.calculate(getArmPos()));
    // }
    // else
    // {
    //   armMotor.stopMotor();
    // }
  }

  // sets the arm speed so long as isnt too high or low
  public void setArmSpeed(double speed){
    if(getArmPos() < ArmConstants.MaxArmMargin && getArmPos() > ArmConstants.MinArmMargin){
      armMotor.set(speed);
    }
    else if(getArmPos() >= ArmConstants.MaxArmMargin)
    {
      if(speed < 0){
        armMotor.set(speed);
      } 
      else 
      {
        armMotor.stopMotor();
      }
    }
    else if(getArmPos() <= ArmConstants.MinArmMargin)
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

  public Command autoSetArmSpeed(double speed){
    return runOnce(
      () -> setArmSpeed(speed)
    );
  }

  // gets the arm position in ticks
  public double getArmPos(){
    return armEncoder.getPosition();
  }

  // can be called to manually make the PID go to our desired goal
  // doesnt have a second failsafe for a goal too far
  public void manualGoToGoal(double goal){
    this.PIDgoal = goal;
    setArmSpeed(armPIDController.calculate(getArmPos(), goal));
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
  //   if(goal < ArmConstants.MaxArmMargin && goal > ArmConstants.MinArmMargin){
  //     armPIDController.setGoal(goal);
  //     goalSet = true;
  //   }
  // }

  // public Command setGoal(double goal){
  //   return runOnce(() -> { 
  //     this.PIDGoal = goal; 
  //   });
  // }

}
