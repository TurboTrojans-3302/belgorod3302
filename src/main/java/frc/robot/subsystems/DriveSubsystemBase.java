// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class DriveSubsystemBase extends SubsystemBase {

    public static final Pose2d defaultStartPosition = new Pose2d(Translation2d.kZero, Rotation2d.kZero);

    private double m_maxSpeed = 0.0;

    //todo upload the field map to the camera?
    protected SwerveDrivePoseEstimator mOdometry;

    public Pose2d getPose() {
        return mOdometry.getEstimatedPosition();
    }

    public void updateOdometry(SwerveModulePosition[] positions) {
        mOdometry.update(Rotation2d.fromDegrees(getHeading()), positions);
    }

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
    public double getHeading(){
        return Units.degreesToRadians(getGyroAngleRadians());
    }

    /**
     * Returns the heading of the robot from the gyro.
     *
     * @return the robot's heading in Radians, from -pi to +pi
     */
    public abstract double getGyroAngleRadians();

    public abstract double getTurnRate();

    public abstract ChassisSpeeds getChassisSpeeds();

    public abstract Double getDistanceToObjectMeters();

    public abstract boolean distanceMeasurmentGood();

    public abstract void resetOdometry(Pose2d pose);

    public abstract double getMaxSpeedLimit();

    public double getMaxSpeed() {
        return m_maxSpeed;
    }

    public void setMaxSpeed() {
        double speed = getSpeed();
        if (speed >= m_maxSpeed) {
            m_maxSpeed = (speed + m_maxSpeed) / 2.0;
        }
    }

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

    public void driveFieldOriented(Translation2d translation, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), -translation.getY(),
                rotation, Rotation2d.fromDegrees(getHeading()));
        drive(speeds);
    }

    public void driveRobotOriented(Translation2d translation, double rotation) {
        ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), -translation.getY(), rotation);
        drive(speeds);
    }

    public double getSpeed() {
        ChassisSpeeds chassisSpeeds = getChassisSpeeds();
        return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
      }
    
      public Translation2d getVelocityVector() {
        ChassisSpeeds chassisSpeeds = getChassisSpeeds();
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

}
