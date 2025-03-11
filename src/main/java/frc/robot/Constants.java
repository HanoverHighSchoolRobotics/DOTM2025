// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    //firebear pathplanner
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.75);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.75);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = true;

    public static final double HorizontalAlignmentSpeed = .2;
    public static final double HorizontalAlignmentkP = 1.5;

    public static final double MaxMoveInRangeSpeed = .2;

    public static final double SlowRotSpeed = .2;
    public static final double VerticalAlignmentSpeed = .2;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kAuxControllerPort = 1;
    public static final double kAuxDeadband = 0.05;

    public static final double kFASTDRIVESPEEDLIMITER = .85;
    public static final double kFASTROTSPEEDLIMITER = .7;

    public static final double kSLOWDRIVESPEEDLIMITER = .2;
    public static final double kSLOWROTSPEEDLIMITER = .2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ElevatorConstants {
    // subsystem constants
    public static final int LeftMotorID = 13;
    public static final int RightMotorID = 12;

    public static final boolean LeftMotorInverted = false;
    public static final boolean RightMotorInvertedFromLeft = true;

    public static final double MaxElevatorMargin = 145;
    public static final double MinElevatorMargin = -5;

    public static final double kP = .3; //  OG was 3; touch this and stuff may start oscillating (dont increase w/o my permission)
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double MaxPIDVelocity = 15; //15
    public static final double MaxPIDAcceleration = 5; //5

    public static final double PIDErrorAllowed = .05;

    // command input constants
    public static final double ElevatorUpSpeed = .7; //.7
    public static final double ElevatorDownSpeed = .6; //.4

    // PID Positions
    // public static final double CoralOnePos = 0;
    public static final double CoralTwoPos = 0;
    public static final double CoralThreePos = 48;
    public static final double CoralFourPos = 132; //132 75 inches top of coral
    public static final double StationPos = 12;
  }
  public static final class CoralIntakeConstants {
    // subsystem constants
    public static final int IntakeMotorID = 15;

    public static final boolean CoralIntakeMotorInverted = false; 

    // command input constants
    public static final double CoralIntakeSpeed = .45;
    public static final double CoralOuttakeSpeed = .3; //.3
  }

  public static final class AlgaeIntakeConstants {
    // subsystem constants
    public static final int IntakeMotorID = 14;

    public static final boolean AlgaeIntakeMotorInverted = false;

    // command input constants
    public static final double AlgaeIntakeSpeed = .5;
  }

  public static final class ArmConstants {
    // subsystem constants
    public static final int ArmMotorID = 11;
    public static final boolean ArmMotorInverted = true;

    public static final double MaxArmMargin = 50000;
    public static final double MinArmMargin = -50000;

    public static final double kP = .05;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double MaxPIDSetSpeed = .15;

    public static final double MaxPIDVelocity = 20;
    public static final double MaxPIDAcceleration = 10;

    public static final double PIDErrorAllowed = .2;

    // command input constants
    public static final double ArmSpeed = .5;

    // PID Positions
    public static final double OutOfTheWayPos = 30;
  }

  public static final class WristConstants {
    // subsystem constants
    public static final int WristMotorID = 9; 
    public static final boolean WristInverted = false; 

    public static final double MaxWristMargin = 1000;
    public static final double MinWristMargin = -1000;
    
    public static final double kP = .05;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double MaxPIDSetSpeed = .4;

    public static final double MaxPIDVelocity = .7; // play around with this - Leighton
    public static final double MaxPIDAcceleration = .3;  // play around with this - Leighton

    public static final double PIDErrorAllowed = .1;
    // command input constants
    public static final double WristSpeed = .35; // Ryan you make all the mistakes - Devon Centeno, March 1st 2025, 11:57 AM EST.
                                                 // Devon you the mistake 😂😂😂 - Ryan Palmer, March 6th 2025, 4:09 PM EST.

  }

  public static final class SecondaryWristConstants {
    // subsystem constants
    public static final int WristMotorID = 20; 
    public static final boolean WristInverted = false; 

    public static final double MaxWristMargin = 1000;
    public static final double MinWristMargin = -1000;
    
    public static final double kP = .05;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double MaxPIDSetSpeed = .4;

    public static final double MaxPIDVelocity = .7; // play around with this - Leighton
    public static final double MaxPIDAcceleration = .3;  // play around with this - Leighton

    public static final double PIDErrorAllowed = .1;
    // command input constants
    public static final double WristSpeed = .35; // Ryan you make all the mistakes - Devon Centeno, March 1st 2025, 11:57 AM EST.
                                                 // Devon you the mistake 😂😂😂 - Ryan Palmer, March 6th 2025, 4:09 PM EST.

    public static final double L4WristPos = .1;
    public static final double L3WristPos = .4;

    
  }

  public static final class LimelightConstants {
    public static final double ROTkP = .035;
    public static final double RANGEkP = .4;

    public static final double HORkP = .01;
    public static final double MaxAlignSpeed = .2;

    public static final double CloseEnoughArea = 4.15;
  }

  public static final class FieldConstants {
    public static final double coralToCoralLength = Units.inchesToMeters(13.25);
    public static final double coralFromCenter = coralToCoralLength / 2;

    public static final double fieldWidth = 8.025;

    public static final double limelightRangeStopToCoralStation = .064;
    public static final double realLimelightRangeStopToCoralStation = .5;
  }

  public static final class ClimberConstants {
    // subsystem constants
    public static final int ClimberMotorID = 10;
    public static final boolean ClimberInverted = false;
    public static final double MaxClimbMargin = 100000;
    public static final double MinClimbMargin = -100000;
    public static final double ClimbUpSpeed = .9;
    public static final double ClimbDownSpeed = .4;
  }

}


  //firebear pathplanner
