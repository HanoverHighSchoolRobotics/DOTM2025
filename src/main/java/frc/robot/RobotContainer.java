// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveClickToAngle;
import frc.robot.commands.LimelightHorizontalAlign;
import frc.robot.commands.LimelightRange;
import frc.robot.commands.ManualMoveHorizontalDistance;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralIntakeSubsystem m_coralIntake = new CoralIntakeSubsystem();
  private final AlgaeIntakeSubsystem m_algaeIntake = new AlgaeIntakeSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_auxController = new XboxController(OIConstants.kAuxControllerPort);

  UsbCamera usbcamera1;
  UsbCamera usbcamera2;

  private final SendableChooser<Command> autoChooser;

  // Pathplanner Command List
  private void configureAutoCommands(){
    NamedCommands.registerCommands(Map.of(
        "CoralIntake",Commands.parallel(
                m_coralIntake.autoAutoCoralIntake(CoralIntakeConstants.CoralIntakeSpeed, 3)),
        "AlgaeIntake",Commands.parallel(
                m_algaeIntake.autoAutoAlgaeIntake(AlgaeIntakeConstants.AlgaeIntakeSpeed, 3))
        // "ArmSetTop",Commands.parallel(
        //         m_arm.autoManualGoToGoal(0) 
        ));
  }


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    usbcamera1 = CameraServer.startAutomaticCapture(0);
    usbcamera2 = CameraServer.startAutomaticCapture(1);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * OIConstants.kFASTDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * OIConstants.kFASTDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * OIConstants.kFASTROTSPEEDLIMITER, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    // Build an auto chooser. This will use Commands.none() as the default option.
    configureAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption("Test Auto (Example Auto)", new PathPlannerAuto("Example Auto"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    JoystickButton aDriverButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    aDriverButton.toggleOnTrue(new RunCommand(
      () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY() * OIConstants.kFASTDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX() * OIConstants.kFASTDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX() * OIConstants.kFASTROTSPEEDLIMITER, OIConstants.kDriveDeadband),
          false),
      m_robotDrive));

    JoystickButton bDriverButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    bDriverButton.toggleOnTrue(new RunCommand(
      () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY() * OIConstants.kSLOWDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX() * OIConstants.kSLOWDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX() * OIConstants.kSLOWROTSPEEDLIMITER, OIConstants.kDriveDeadband),
          true),
      m_robotDrive));

    JoystickButton xDriverButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    xDriverButton.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    JoystickButton yDriverButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    yDriverButton.whileTrue(new LimelightRange(m_robotDrive));

    JoystickButton startDriverButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
    startDriverButton.whileTrue(new DriveClickToAngle(m_robotDrive));

    JoystickButton backDriverButton = new JoystickButton(m_driverController, XboxController.Button.kBack.value);
    backDriverButton.whileTrue(new LimelightHorizontalAlign(m_robotDrive));

    JoystickButton rightBumperDriverButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    rightBumperDriverButton.whileTrue(new ManualMoveHorizontalDistance(m_robotDrive, "Right", FieldConstants.coralFromCenter));

    JoystickButton leftBumperDriverButton = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    leftBumperDriverButton.whileTrue(new ManualMoveHorizontalDistance(m_robotDrive, "Left", FieldConstants.coralFromCenter));

    JoystickButton rightTriggerDriverButton = new JoystickButton(m_driverController, Button.kR1.value);

    JoystickButton leftTriggerDriverButton = new JoystickButton(m_driverController, Button.kL1.value);
    
    JoystickButton aAuxButton = new JoystickButton(m_auxController, XboxController.Button.kA.value);
      aAuxButton.onTrue(m_coralIntake.autoCoralIntake(CoralIntakeConstants.CoralIntakeSpeed))
      .onFalse(m_coralIntake.autoCoralIntake(0));

    JoystickButton bAuxButton = new JoystickButton(m_auxController, XboxController.Button.kB.value);
      bAuxButton.onTrue(m_algaeIntake.autoAlgaeIntake(AlgaeIntakeConstants.AlgaeIntakeSpeed))
      .onFalse(m_algaeIntake.autoAlgaeIntake(0));

    JoystickButton xAuxButton = new JoystickButton(m_auxController, XboxController.Button.kX.value);
      xAuxButton.onTrue(m_elevator.autoSetElevatorSpeed(ElevatorConstants.ElevatorSpeed))
      .onFalse(m_elevator.autoSetElevatorSpeed(0));

    JoystickButton yAuxButton = new JoystickButton(m_auxController, XboxController.Button.kY.value);
      yAuxButton.onTrue(m_elevator.autoSetElevatorSpeed(-1 * ElevatorConstants.ElevatorSpeed))
      .onFalse(m_elevator.autoSetElevatorSpeed(0));

    JoystickButton startAuxButton = new JoystickButton(m_auxController, XboxController.Button.kStart.value);
      startAuxButton.onTrue(m_arm.autoSetArmSpeed(ArmConstants.ArmSpeed))
      .onFalse(m_arm.autoSetArmSpeed(0));

    JoystickButton backAuxButton = new JoystickButton(m_auxController, XboxController.Button.kBack.value);
      backAuxButton.onTrue(m_arm.autoSetArmSpeed(-1 * ArmConstants.ArmSpeed))
      .onFalse(m_arm.autoSetArmSpeed(0));

    JoystickButton rightBumperAuxButton = new JoystickButton(m_auxController, XboxController.Button.kRightBumper.value);

    JoystickButton leftBumperAuxButton = new JoystickButton(m_auxController, XboxController.Button.kLeftBumper.value);

    JoystickButton rightTriggerAuxButton = new JoystickButton(m_auxController, Button.kR1.value);

    JoystickButton leftTriggerAuxButton = new JoystickButton(m_auxController, Button.kL1.value);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

    return autoChooser.getSelected();
  }
}