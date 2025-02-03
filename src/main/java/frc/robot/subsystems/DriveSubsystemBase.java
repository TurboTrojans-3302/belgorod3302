// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class DriveSubsystemBase extends SubsystemBase {
    public abstract Pose2d getPose();

    /**
     * X and Y directions are relative to the robot
     * @param x scaled speed in the x direction [-1, 1]
     * @param y scaled speed in the y direction [-1, 1]
     * @param turn scaled speed for turning [-1, 1]
     *
     */
    public abstract void driveRobotOriented(Double x, Double y, Double turn);
    
    /**
     * X and Y directions are relative to the field
     * @param x scaled speed in the x direction [-1, 1]
     * @param y scaled speed in the y direction [-1, 1]
     * @param turn scaled speed for turning [-1, 1]
     *
     */
    public abstract void driveFieldOriented(Double x, Double y, Double turn);
    
    public abstract double getHeading();

    public abstract double getTurnRate();

    public abstract double getSpeed();

    public abstract Double getDistanceToObjectMeters();

    public abstract boolean distanceMeasurmentGood();

    public abstract void resetOdometry(Pose2d pose);

    public abstract double getMaxSpeedMetersPerSecond();

    public abstract double turnToHeading(double heading);

    public abstract void drive(ChassisSpeeds speeds);

    public abstract void stop();
        
    public void driveHeadingField(Translation2d translationMetersPerSecond, double heading) {
        double yawCommand = turnToHeading(heading);
        driveFieldOriented(translationMetersPerSecond, yawCommand);
    }

    
    public void driveHeadingRobot(Translation2d translationMetersPerSecond, double heading) {
        double yawCommand = turnToHeading(heading);
        driveRobotOriented(translationMetersPerSecond, yawCommand);
    }

    public void driveFieldOriented(Translation2d translation, double rotation){
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), -translation.getY(),
                                                                     rotation, Rotation2d.fromDegrees(getHeading()));
        drive(speeds);
    }
    
    public void driveRobotOriented(Translation2d translation, double rotation) {
        ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), -translation.getY(), rotation);
        drive(speeds);
    }

}
