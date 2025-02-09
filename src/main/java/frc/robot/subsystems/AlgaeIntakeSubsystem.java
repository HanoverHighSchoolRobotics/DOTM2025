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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Made by Vincent Zatelli. */
  /** Creates a new ExampleSubsystem. */
  SparkMax intakeMotor;

  public AlgaeIntakeSubsystem() {
    intakeMotor = new SparkMax(AlgaeIntakeConstants.IntakeMotorID,MotorType.kBrushless);

    intakeMotor.configure(Configs.AlgaeIntake.algaeIntakeMotorConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public Command autoAlgaeIntake(double speed) {
    return runOnce(
      () -> setIntake(speed)
    );
  }

  public Command autoAutoAlgaeIntake(double speed, double timeOut){
    return run(
      () -> setIntake(speed)
      ).withTimeout(timeOut);
  }
}
