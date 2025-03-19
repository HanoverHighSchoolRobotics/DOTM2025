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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  // SparkMax rightElevatorMotor;
  SparkMax leftElevatorMotor;

  RelativeEncoder elevatorEncoder;

  ProfiledPIDController elevatorPIDController = new ProfiledPIDController(
    ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
    new TrapezoidProfile.Constraints(ElevatorConstants.MaxPIDVelocity, ElevatorConstants.MaxPIDAcceleration));

  private double PIDgoal;
  private boolean goalSet;

  private double clampSpeed;

  public ElevatorSubsystem() {
    leftElevatorMotor = new SparkMax(ElevatorConstants.LeftMotorID, MotorType.kBrushless);
    // rightElevatorMotor = new SparkMax(ElevatorConstants.RightMotorID, MotorType.kBrushless);

    leftElevatorMotor.configure(Configs.Elevator.leftElevatorMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    // rightElevatorMotor.configure(Configs.Elevator.rightElevatorMotorConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    elevatorEncoder = leftElevatorMotor.getEncoder();
    
    elevatorEncoder.setPosition(0);

    goalSet = false;

    clampSpeed = ElevatorConstants.absMaxPIDSpeed;

    elevatorPIDController.setTolerance(ElevatorConstants.PIDErrorAllowed);
  }

  @Override
  public void periodic() {

    if(goalSet && !elevatorPIDController.atSetpoint()){
      setElevatorSpeed(MathUtil.clamp(elevatorPIDController.calculate(getPosition(), PIDgoal), -1 * clampSpeed, clampSpeed));
    }
    else if (goalSet)
    {
      leftElevatorMotor.stopMotor();
    }
    //test to make it so when the pid gets enabled, it doesnt go to last setpoint
    else
    {
      PIDgoal = getPosition();
    }

    SmartDashboard.putNumber("Elevator Encoder Reading", getPosition());
    SmartDashboard.putNumber("Calculated Elevator Speed",elevatorPIDController.calculate(getPosition(), PIDgoal));
    SmartDashboard.putNumber("Displayed Elevator Speed", MathUtil.clamp(elevatorPIDController.calculate(getPosition(), PIDgoal), -.75, .75));
    SmartDashboard.putNumber("Elevator Goal", PIDgoal);
  }

  // sets the elevator speed so long as isnt too high or low
  public void setElevatorSpeed(double speed){
    double position = getPosition();

    if(position < ElevatorConstants.MaxElevatorMargin && position > ElevatorConstants.MinElevatorMargin){
      leftElevatorMotor.set(speed);
    }
    else if(position >= ElevatorConstants.MaxElevatorMargin)
    {
      if(speed < 0){
        leftElevatorMotor.set(speed);
      } 
      else 
      {
        leftElevatorMotor.stopMotor();
      }
    }
    else if(position <= ElevatorConstants.MinElevatorMargin)
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

  // only use for testing or resetting the robot position
  public Command setElevatorSpeedNoLimitsCmd(double speed){
    return run(
      () -> leftElevatorMotor.set(speed)
    );
  }

  public Command SetElevatorSpeedCmd(double speed){
    return runOnce(
      () -> setElevatorSpeed(speed)
    );
  }

  // gets the elevator position in ticks
  public double getPosition(){
    return elevatorEncoder.getPosition();
  }

  // sets a goal and for as long as the robot is on
  // the elevator will do its best to reach this goal and is used through periodic
  public void setGoal(double goal){
    // if(goal == ElevatorConstants.CoralFourPos){
    //   clampSpeed = .3; //float slower speed 
    // } else {
    //   clampSpeed = ElevatorConstants.absMaxPIDSpeed;
    // }
    if(goal <= ElevatorConstants.MaxElevatorMargin && goal >= ElevatorConstants.MinElevatorMargin){
      this.PIDgoal = goal;
      elevatorPIDController.setGoal(goal);
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

