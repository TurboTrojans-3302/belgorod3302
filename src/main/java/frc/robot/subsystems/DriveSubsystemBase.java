// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class DriveSubsystemBase extends SubsystemBase {
    public abstract Pose2d getPose();

    public abstract void drive(Translation2d translation, double rotation);

    public abstract void driveHeadingRobot(Translation2d translation, double rotation);

    public abstract void driveHeadingField(Translation2d translation, double rotation);

    public abstract void drive(Translation2d translation);

    public abstract double getHeading();

    public abstract double getTurnRate();

    public abstract double getSpeed();

    public abstract Double getDistanceToObjectMeters();

    public abstract boolean distanceMeasurmentGood();

    public abstract void resetOdometry(Pose2d pose);

    public abstract double getMaxSpeedMetersPerSecond();
}
