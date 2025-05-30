package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SecondaryWristConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.AlgaeIntakeConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to tune them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);

        }
    }

    public static final class Elevator {
        public static final SparkMaxConfig leftElevatorMotorConfig = new SparkMaxConfig();
        // public static final SparkMaxConfig rightElevatorMotorConfig = new SparkMaxConfig();

        static {
            leftElevatorMotorConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(ElevatorConstants.LeftMotorInverted);
        //     rightElevatorMotorConfig
        //             .idleMode(IdleMode.kBrake)
        //             .follow(ElevatorConstants.LeftMotorID, ElevatorConstants.RightMotorInvertedFromLeft);
        }
    }
    public static final class AlgaeIntake {
        public static final SparkMaxConfig algaeIntakeMotorConfig = new SparkMaxConfig();

        static {
                algaeIntakeMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(AlgaeIntakeConstants.AlgaeIntakeMotorInverted);
        }
    }
    
    public static final class CoralIntake {
        public static final SparkMaxConfig coralIntakeMotorConfig = new SparkMaxConfig(); 

        static {
                coralIntakeMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(CoralIntakeConstants.CoralIntakeMotorInverted);
        }
    }

    public static final class Arm {
        public static final SparkMaxConfig armMotorConfig = new SparkMaxConfig(); 

        static {
                armMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ArmConstants.ArmMotorInverted);
        }

    }    
    
    public static final class Wrist {
        public static final SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

        static {
                wristMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(WristConstants.WristInverted);
        }

    }

    public static final class SecondaryWrist {
        public static final SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

        static {
                wristMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(SecondaryWristConstants.WristInverted);
        }

    }

    public static final class Climber {
        public static final SparkMaxConfig climberMotorConfig = new SparkMaxConfig();


        static {
                climberMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ClimberConstants.ClimberInverted);
        }
    }
}