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
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  SparkMax climbMotor;

  RelativeEncoder climbEncoder;

  public ClimberSubsystem() {
    climbMotor = new SparkMax(ClimberConstants.ClimberMotorID, MotorType.kBrushless);

    climbMotor.configure(Configs.Climber.climberMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    climbEncoder = climbMotor.getEncoder();
    
    climbEncoder.setPosition(0);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Encoder", getPosition());
  }

  // sets the climb speed so long as isnt too high or low
  public void setClimbSpeed(double speed){
    double position = getPosition();

    if(position < ClimberConstants.MaxClimbMargin && position > ClimberConstants.MinClimbMargin){
      climbMotor.set(speed);
    }
    else if(position >= ClimberConstants.MaxClimbMargin)
    {
      if(speed < 0){
        climbMotor.set(speed);
      } 
      else 
      {
        climbMotor.stopMotor();
      }
    }
    else if(position <= ClimberConstants.MinClimbMargin)
    {
      if(speed > 0){
        climbMotor.set(speed);
      } 
      else 
      {
        climbMotor.stopMotor();
      }
    }
    else
    {
      climbMotor.stopMotor();
    }
  }

  // takes in the difference in trigger values, follow the math gang
  public Command setClimbSpeedCmd(double speed){
    return runOnce(
      () -> {
        setClimbSpeed(speed);
      }
    );
  }

  // gets the climb position in ticks
  public double getPosition(){
    return climbEncoder.getPosition();
  }
}

