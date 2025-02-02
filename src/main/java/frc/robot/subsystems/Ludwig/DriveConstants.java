// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Ludwig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Math.toRadians(17.5);
    public static final double kFrontRightChassisAngularOffset = Math.toRadians(185.7);
    public static final double kBackLeftChassisAngularOffset = Math.toRadians(0.0);
    public static final double kBackRightChassisAngularOffset = Math.toRadians(58.0);

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 1;
    public static final int kRearRightTurningCanId = 7;

    public static final int kDXSensorCanId = 0;

    public static final boolean kGyroReversed = false;
    public static final double headingP = 0.02;
    public static final double headingI = 0.0001;
    public static final double headingD = 0;

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}
