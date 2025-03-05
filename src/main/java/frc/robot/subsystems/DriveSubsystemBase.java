// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class DriveSubsystemBase extends SubsystemBase {

    

    //todo upload the field map to the camera?


    /**
     * X and Y directions are relative to the robot
     * 
     * @param x    scaled speed in the x direction [-1, 1]
     * @param y    scaled speed in the y direction [-1, 1]
     * @param turn scaled speed for turning [-1, 1]
     *
     */
    public abstract void driveRobotOriented(Double x, Double y, Double turn);

    /**
     * X and Y directions are relative to the field
     * 
     * @param x    scaled speed in the x direction [-1, 1]
     * @param y    scaled speed in the y direction [-1, 1]
     * @param turn scaled speed for turning [-1, 1]
     *
     */
    public abstract void driveFieldOriented(Double x, Double y, Double turn);

    /**
     * Returns the heading of the robot from the gyro.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public abstract double getGyroAngleDegrees();

    public abstract void setGyroAngleDeg(double angle);

    /**
     * Returns the heading of the robot from the gyro.
     *
     * @return the robot's heading in Radians, from -pi to +pi
     */
    public abstract double getGyroAngleRadians();

    public abstract double getTurnRate();

    public abstract ChassisSpeeds getChassisSpeeds();

    public abstract double getMaxSpeedLimit();

    public abstract SwerveModulePosition[] getSwerveModulePositions();

    public abstract SwerveDriveKinematics getKinematics();


    public abstract double turnToHeadingDegrees(double heading);

    public abstract void drive(ChassisSpeeds speeds, Translation2d centerOfRotation);

    public abstract void testSetAll(double voltage, double angleRadians);

    

    

}
