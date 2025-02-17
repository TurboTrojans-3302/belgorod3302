// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Eddie;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class DriveConstants {

        public static final double TRACKWIDTH = 19.5 * 0.0254; // distance between the left and right wheels
        public static final double WHEELBASE = 23.5 * 0.0254; // front to back distance

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        new Translation2d(WHEELBASE / 2.0, TRACKWIDTH / 2.0), // front left
                        new Translation2d(WHEELBASE / 2.0, -TRACKWIDTH / 2.0), // front right
                        new Translation2d(-WHEELBASE / 2.0, TRACKWIDTH / 2.0), // back left
                        new Translation2d(-WHEELBASE / 2.0, -TRACKWIDTH / 2.0) // back right
        );

        public static final double SLEW_LIMIT_TRANSLATION = 10.0; // m/s^2
        public static final double SLEW_LIMIT_ROTATION =  10.0;  // rad/s^2     
        public static final double AutonSpeedLimit = 4.0;
        public static final double AutonAccelLimit = 2.0;
        public static final double kMaxRotation = 8.0;
        public static final double kMaxSpeedMetersPerSecond = 12.0;
        public static final double ORBIT_SPEED = 3.0;
}
