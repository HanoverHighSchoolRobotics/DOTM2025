// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new Subsystem. */
  //Made by JonMichael Shadoyan
  //Add motor instantiations here
  SparkMax intakeMotor;


  public CoralIntakeSubsystem() {
  //Assign these motors a value here
    intakeMotor = new SparkMax(CoralIntakeConstants.IntakeMotorID,MotorType.kBrushless);

    intakeMotor.configure(Configs.CoralIntake.coralIntakeMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public Command autoCoralIntake(double speed){
    return runOnce(
      () -> setIntake(speed)
    );
  }

  public Command autoAutoCoralIntake(double speed, double timeOut){
    return run(
      () -> setIntake(speed)
      ).withTimeout(timeOut);
  }
}