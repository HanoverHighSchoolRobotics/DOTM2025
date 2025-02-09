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
import frc.robot.Configs.Elevator;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax leftElevatorMotor;
  SparkMax rightElevatorMotor;

  RelativeEncoder elevatorEncoder;

  ProfiledPIDController elevatorPIDController = new ProfiledPIDController(
  ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
    new TrapezoidProfile.Constraints(ElevatorConstants.MaxPIDVelocity, ElevatorConstants.MaxPIDAcceleration));

  private double PIDgoal;
  private boolean goalSet;

  public ElevatorSubsystem() {
    leftElevatorMotor = new SparkMax(ElevatorConstants.LeftMotorID, MotorType.kBrushless);
    rightElevatorMotor = new SparkMax(ElevatorConstants.RightMotorID, MotorType.kBrushless);

    leftElevatorMotor.configure(Configs.Elevator.leftElevatorMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
    rightElevatorMotor.configure(Configs.Elevator.rightElevatorMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    elevatorEncoder = leftElevatorMotor.getEncoder();
    
    elevatorEncoder.setPosition(0);

    goalSet = false;
  }

  @Override
  public void periodic() {
    // only uncomment if we are sure the profiled PID can work at all times, also add a failsafe button
    // if(getElevatorPos() < PIDgoal - ElevatorConstants.PIDErrorAllowed && getElevatorPos() > PIDgoal + 
    // ElevatorConstants.PIDErrorAllowed && goalSet){
    //   setElevatorSpeeds(elevatorPIDController.calculate(getElevatorPos()));
    // }
    // else
    // {
    //   leftElevatorMotor.stopMotor();
    // }
  }

  public void setElevatorSpeeds(double speed){
    if(getElevatorPos() < ElevatorConstants.MaxElevatorMargin && getElevatorPos() > ElevatorConstants.MinElevatorMargin){
      leftElevatorMotor.set(speed);
    }
    else if(getElevatorPos() >= ElevatorConstants.MaxElevatorMargin)
    {
      if(speed < 0){
        leftElevatorMotor.set(speed);
      } 
      else 
      {
        leftElevatorMotor.stopMotor();
      }
    }
    else if(getElevatorPos() <= ElevatorConstants.MinElevatorMargin)
    {
      if(speed > 0){
        leftElevatorMotor.set(speed);
      } 
      else 
      {
        leftElevatorMotor.stopMotor();
      }
    }
    else
    {
      leftElevatorMotor.stopMotor();
    }
  }

  public Command autoSetElevatorSpeed(double speed){
    return runOnce(
      () -> setElevatorSpeeds(speed)
    );
  }

  // gets the elevator position in ticks
  public double getElevatorPos(){
    return elevatorEncoder.getPosition();
  }

  // can be called to manually make the PID go to our desired goal
  // doesnt have a second failsafe for a goal too far
  public void manualGoToGoal(double goal){
    this.PIDgoal = goal;
    setElevatorSpeeds(elevatorPIDController.calculate(getElevatorPos(), goal));
    goalSet = true;
  }

  public Command autoManualGoToGoal(double goal){
    return runOnce(
      () -> manualGoToGoal(goal)
    );
  }

  // sets a goal and for as long as the robot is on
  // the elevator will do its best to reach this goal and is used through periodic
  // public void setGoal(double goal){
  //   this.PIDgoal = goal;
  //   if(goal < ElevatorConstants.MaxElevatorMargin && goal > ElevatorConstants.MinElevatorMargin){
  //     elevatorPIDController.setGoal(goal);
  //     goalSet = true;
  //   }
  // }

  // public Command setGoal(double goal){
  //   return runOnce(() -> { 
  //     this.PIDGoal = goal; 
  //   });
  // }


}
