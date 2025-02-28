// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

  public final class CanIds {
    public static final int DX_SENSOR_CAN_ID = 0;

    // Eddie DriveTrain
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 2;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 4;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 5;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 6;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 7;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 8;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 9;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 10;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 11;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 12;

    // Ludwig DriveTrain
    public static final int kFrontRightTurningCanId = 1;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kFrontLeftTurningCanId = 3;
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearRightTurningCanId = 7;
    public static final int kRearRightDrivingCanId = 8;
    // 9
    // 10
    // 11
    // 12
    public static final int kIntakeMotorCanId = 13;    
    public static final int kLeftMotorElevatorCanId = 14;
    public static final int kRightMotorElevatorCanId = 15;
    public static final int kGripperMotorCanId = 16;
    public static final int kGripperExtensionMotorCanId = 17;
    public static final int kClimberLeftMotorCanId = 18;
    public static final int kClimberRightMotorCanId = 19;
    public static final int intakeArmMotorID = 20;
  }

  public static final class DigitalIO {
        public static final int kElevatorHighLimitSwitchId = 0;
        public static final int kElevatorLowLimitSwitchId = 1;
        public static final int kIntakeLimitSwitchId = 2;
        public static final int kIntakeArmLimitSwitchId = 3;
        public static final int kGripperClosedSwitchId = 4;
        public static final int kGripperFullyRetractedSwitchId = 5;
        public static final int kGripperObjectDetectedSwitchId = 6;
        public static final int kClimberLimitSwitchId = 7;
      }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kCopilotControllerPort = 1;
    public static final int kButtonBoardPort = 2;
    public static final class ButtonBox {
      public static final int Left1 = 0;
      public static final int Right1 = 1;
      public static final int Left2 = 2;
      public static final int Right2 = 3;
      public static final int Left3 = 4;
      public static final int Right3 = 5;
      public static final int Left4 = 6;
      public static final int Right4 = 7;
      public static final int Esc = 8;
      public static final int Enter = 9;
      public static final int EngineStart = 10;
      public static final int SafetySwitch = 11;
      public static final int Switch1Up = 12;
      public static final int Switch1Down = 13;
      public static final int Switch2Up = 14;
      public static final int Switch2Down = 15;
      public static final int Switch3Up = 16;
      public static final int Switch3Down = 17;
      public static final int Switch4Up = 18;
      public static final int Switch4Down = 19;
      public static final int LeftKnobCW = 20;
      public static final int LeftKnobCCW = 21;
      public static final int RightKnobCCW = 22;
      public static final int RightKnobCW = 23;

      public static final double StickUp = -1.0;
      public static final double StickUpRight = -0.71429;
      public static final double StickRight = -0.42857;
      public static final double StickDownRight = -0.14286;
      public static final double StickDown = 0.14286;
      public static final double StickDownLeft = 0.42857;
      public static final double StickLeft = 0.71429;
      public static final double StickUpLeft = 1.0;
    }
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
    public static final double kDistanceTolerance = 0.050;
    public static final double kHeadingTolerance = 2.0; // degrees
  }

  public static final class FieldConstants {
    public static final Pose2d ZeroZero = new Pose2d(0.0, 0.0, new Rotation2d());
    // for the station with april tag 13 these values both stay positive
    public static final double poseOffsetStationRightX = 0.5;
    public static final double poseOffsetStationRightY = 0.5;
    //relative in the y-direction
    public static final double yOffsetReefPoleLeft = 0.3;
    public static final double yOffsetReefPoleRight = -0.3;
  }

  public static final class LimelightConstants {
    public static final String name = "limelight";

    public static final class Offset {
      public static final double forward = 0.2985;
      public static final double side = 0.0;
      public static final double up = 0.5;
      public static final double roll = 0.0;
      public static final double pitch = 0.0;
      public static final double yaw = 0.0;
    }

    public static final class PipelineIdx {
      public static final int AprilTag = 0;
      public static final int NeuralDetector = 1;
      public static final int NeuralClassifer = 2;
    }
  }

  public static final class ElevatorConstants {
    // position in motor rotations for different levels of the reef, all random
    // numbers that need to be tested of course
    public static final double kGround = 0;
    public static final double kLevel1Trough = 50;
    public static final double kLevel2 = 75;
    public static final double kLevel3 = 100;
    public static final double kLevel4 = 125;
    // honestly the processor probably wouldn't require the elevator to move at all
    // from the start position, but if it is already up it could be useful to have a
    // preset position
    public static final double kProcessor = 15;

    public static final double kElevatorPrecisionControlSpeed = 0.15;
    public static final double kElevatorAutoSpeedToLevel = 0.4;
    public static final double kElevatorMaxSpeed = 0.75;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static final class IntakeConstants {
    public static final int intakeLimitSwitchID = 2;
    public static final double intakeSpeedMax = 0.75;
    public static final double intakeSpeedAuto = 0.4;
    public static final double kI = 0.0;
    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double intakeSpeedMin = -0.75;
    public static final int armEncoderDInput = 0;
    public static final double armAngleOffset = 0;
    public static final double MaxArmAngle = 90.0;
    public static final double MinArmAngle = 0.0;
    public static final double inSpeed = 0;
    public static final double outSpeed = 0;
  }

  public static final class ClimberConstants {
    public static final int leftClimberID = 13;
    public static final int rightClimberID = 14;
    public static final int climberLimitSwitchID = 3;
    public static double climberMaxSpeed = 0.5;
    public static double climberAutoSpeed = 0.3;

  }

  public static final class GripperConstants {
    public static final int gripperMotorID = 16;
    public static final int gripperExtensionMotorID = 17;
    public static final int limitSwitchID = 5;
    public static final double gripperExtendedPosition = 100.0;
    // only one speed necessary
    public static final double gripperMotorSpeed = 0.6;
    public static final double gripperExtensionSpeed = 0.5;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double openPosition = 100.0;
    public static final double closedPosition = 0.0;
  }

  public static final int BLINKIN_LED_PWM_CHANNEL = 0;
}
