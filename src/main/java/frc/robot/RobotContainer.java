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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
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
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

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
  private final WristSubsystem m_wrist = new WristSubsystem();
  private final ClimberSubsystem m_climb = new ClimberSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_auxController = new CommandXboxController(OIConstants.kAuxControllerPort);

  UsbCamera usbcamera1;
  UsbCamera usbcamera2;

  private final SendableChooser<Command> autoChooser;

  // Pathplanner Command List
  private void configureAutoCommands(){
    NamedCommands.registerCommands(Map.of(
          "CoralIntake", Commands.parallel(
                m_coralIntake.autoCoralIntake(CoralIntakeConstants.CoralIntakeSpeed, 3)),
          "CoralOuttake", Commands.parallel(
                m_coralIntake.autoCoralIntake(-1 * CoralIntakeConstants.CoralIntakeSpeed, 3)),
          "AlgaeIntake", Commands.parallel(
                m_algaeIntake.autoAlgaeIntake(AlgaeIntakeConstants.AlgaeIntakeSpeed, 3)),
          "AlgaeOuttake", Commands.parallel(
                m_algaeIntake.autoAlgaeIntake(-1 * AlgaeIntakeConstants.AlgaeIntakeSpeed, 3)),
          "MoveArmOut", Commands.parallel(
                m_arm.setGoalCmd(ArmConstants.OutOfTheWayPos)),
          "ElevatorToL4", Commands.parallel(
                m_elevator.setGoalCmd(ElevatorConstants.CoralFourPos)),
          "ElevatorToL3", Commands.parallel(
                m_elevator.setGoalCmd(ElevatorConstants.CoralThreePos)),
          "ElevatorToL2", Commands.parallel(
                m_elevator.setGoalCmd(ElevatorConstants.CoralTwoPos)),
          "ElevatorToStation", Commands.parallel(
                m_elevator.setGoalCmd(ElevatorConstants.StationPos))
            ) 
        );
        // "ArmSetTop",Commands.parallel(
        //         m_arm.autoManualGoToGoal(0) 
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

    m_arm.setDefaultCommand(
      new RunCommand(
        () -> m_arm.setArmSpeedVoidCmd(MathUtil.applyDeadband(m_driverController.getLeftY() * ArmConstants.ArmSpeed, OIConstants.kAuxDeadband)), 
        m_arm));

    m_wrist.setDefaultCommand(
      new RunCommand(
        () -> m_wrist.setWristSpeedVoidCmd(MathUtil.applyDeadband(m_driverController.getRightY() * WristConstants.WristSpeed, OIConstants.kAuxDeadband)), 
        m_wrist));

    // Build an auto chooser. This will use Commands.none() as the default option.
    configureAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption("Test Auto (Example Auto)", new PathPlannerAuto("Example Auto"));
    autoChooser.addOption("Auto 6 In Dashboard", new PathPlannerAuto("Auto6"));

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
    
    // JoystickButton aDriverButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    m_driverController.a()
    .toggleOnTrue(new RunCommand(
      () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY() * OIConstants.kFASTDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX() * OIConstants.kFASTDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX() * OIConstants.kFASTROTSPEEDLIMITER, OIConstants.kDriveDeadband),
          false),
      m_robotDrive));

    m_driverController.b()
    .toggleOnTrue(new RunCommand(
      () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY() * OIConstants.kSLOWDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX() * OIConstants.kSLOWDRIVESPEEDLIMITER, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX() * OIConstants.kSLOWROTSPEEDLIMITER, OIConstants.kDriveDeadband),
          true),
      m_robotDrive));

    m_driverController.x()
    // resets gyro
    .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    m_driverController.y()
    // gets in range (close to) the april tag
    .whileTrue(new LimelightRange(m_robotDrive));

    m_driverController.start()
    // clicks to an angle on a hexagon that you are closest to
    .whileTrue(new DriveClickToAngle(m_robotDrive));

    m_driverController.back()
    // aligns horizontally with the april tag
    .whileTrue(new LimelightHorizontalAlign(m_robotDrive));

    m_driverController.rightBumper()
    // moves right of the april tag to be in line with the coral from the center
    .whileTrue(new ManualMoveHorizontalDistance(m_robotDrive, "Right", FieldConstants.coralFromCenter));

    m_driverController.leftBumper()
    // moves left of the april tag to be in line with the coral from the center
    .whileTrue(new ManualMoveHorizontalDistance(m_robotDrive, "Left", FieldConstants.coralFromCenter));

    // m_driverController.leftStick()

    // m_driverController.rightStick()

    m_driverController.rightTrigger(.5)

    .whileTrue(m_climb.setClimbSpeedCmd(ClimberConstants.ClimbUpSpeed));

    m_driverController.leftTrigger(.5)

    .whileTrue(m_climb.setClimbSpeedCmd(-1 * ClimberConstants.ClimbDownSpeed));
    
    // m_auxController.a()
    // set wrist speed positive
      // .whileTrue(m_wrist.SetWristSpeedCmd(WristConstants.WristSpeed))
      // .onFalse(m_wrist.SetWristSpeedCmd(0));
    // set elevator to a setpoint
      // aAuxButton.onTrue(m_elevator.setGoalCmd(ElevatorConstants.CoralOnePos));

    // m_auxController.b()
    // set wrist speed negative
      // .whileTrue(m_wrist.SetWristSpeedCmd(-1 * WristConstants.WristSpeed))
      // .onFalse(m_wrist.SetWristSpeedCmd(0));
    //set elevator to a setpoint
      // bAuxButton.onTrue(m_elevator.setGoalCmd(ElevatorConstants.CoralTwoPos));

    m_auxController.x()
    // set elevator manual positive
      .whileTrue(m_elevator.SetElevatorSpeedCmd(ElevatorConstants.ElevatorSpeed))
      .onFalse(m_elevator.SetElevatorSpeedCmd(0));
    // set elevator to a setpoint
      // xAuxButton.onTrue(m_elevator.setGoalCmd(ElevatorConstants.CoralThreePos));

    m_auxController.y()
    // set elevator manual negative
      .onTrue(m_elevator.SetElevatorSpeedCmd(-1 * ElevatorConstants.ElevatorSpeed))
      .onFalse(m_elevator.SetElevatorSpeedCmd(0));
    // set elevator to a setpoint
      // yAuxButton.onTrue(m_elevator.setGoalCmd(ElevatorConstants.CoralFourPos));

    // m_auxController.start()
    // set arm manual positive
      // .onTrue(m_arm.SetArmSpeedCmd(ArmConstants.ArmSpeed))
      // .onFalse(m_arm.SetArmSpeedCmd(0));
    // set elevator to a setpoint
      // startAuxButton.onTrue(m_elevator.setGoalCmd(ElevatorConstants.StationPos));

    // m_auxController.back()
    // set arm manual negative
      // .whileTrue(m_arm.SetArmSpeedCmd(-1 * ArmConstants.ArmSpeed))
      // .onFalse(m_arm.SetArmSpeedCmd(0));
    // set arm to a setpoint
      // backAuxButton.toggleOnTrue(m_arm.setGoalCmd(ArmConstants.HighPos))
      // .toggleOnFalse(m_arm.setGoalCmd(ArmConstants.LowPos));

    m_auxController.rightBumper()
    // sets the algae intake inwards
      .whileTrue(m_algaeIntake.AlgaeIntakeCmd(AlgaeIntakeConstants.AlgaeIntakeSpeed))
      .onFalse(m_algaeIntake.AlgaeIntakeCmd(0));

    m_auxController.leftBumper()
    // sets the algae intake outwards
      .whileTrue(m_algaeIntake.AlgaeIntakeCmd(-1 * AlgaeIntakeConstants.AlgaeIntakeSpeed))
      .onFalse(m_algaeIntake.AlgaeIntakeCmd(0));

    // m_auxController.rightStick()
    // sets the coral intake inwards
    //   .whileTrue(m_coralIntake.CoralIntakeCmd(CoralIntakeConstants.CoralIntakeSpeed))
    //   .onFalse(m_coralIntake.CoralIntakeCmd(0));

    // m_auxController.leftStick()
    // sets the coral intake outwards
      // .whileTrue(m_coralIntake.CoralIntakeCmd(-1 * CoralIntakeConstants.CoralIntakeSpeed))
      // .onFalse(m_coralIntake.CoralIntakeCmd(0));

    m_auxController.rightTrigger(.5)
    // sets the coral intake inwards
      .whileTrue(m_coralIntake.CoralIntakeCmd(CoralIntakeConstants.CoralIntakeSpeed))
      .onFalse(m_coralIntake.CoralIntakeCmd(0));

    m_auxController.leftTrigger(.5)
    // sets the coral intake outwards
      .whileTrue(m_coralIntake.CoralIntakeCmd(-1 * CoralIntakeConstants.CoralIntakeSpeed))
      .onFalse(m_coralIntake.CoralIntakeCmd(0));

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