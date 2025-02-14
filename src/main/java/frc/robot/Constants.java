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

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kCopilotControllerPort = 1;
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

  public static final int DX_SENSOR_CAN_ID = 0;

  public static final class FieldConstants {
    public static final Pose2d ZeroZero = new Pose2d(0.0, 0.0, new Rotation2d());
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
  }

  public static final class ElevatorConstants {
    // theoretical for now
    public static final int kLeftMotorElevatorCanId = 10;
    public static final int kRightMotorElevatorCanId = 11;
    // dio roborio
    public static final int kElevatorHighLimitSwitchId = 0;
    public static final int kElevatorLowLimitSwitchId = 1;
    // position in motor rotations for different levels of the reef, all random
    // numbers that need to be tested of course
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
  }

  public static final class IntakeConstants {
    public static final int intakeMotorID = 12;
    public static final int intakeArmMotorID = 13;
    public static final int intakeLimitSwitchID = 2;
    public static final double intakeSpeedMax = 0.75;
    public static final double kI = 0.0;
    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double intakeSpeedMin = -0.75;
    public static final int armEncoderDInput = 0;
    public static final double armAngleOffset = 0;
    public static final double MaxArmAngle = 90.0;
    public static final double MinArmAngle = 0.0;
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
    //only one speed necessary
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
