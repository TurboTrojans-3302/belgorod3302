// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new YDriveSubsystem. */

  public static final Pose2d defaultStartPosition = new Pose2d(Translation2d.kZero, Rotation2d.kZero);
  public SwerveDrive m_SwerveDrive;

  public DriveSubsystem() {
    this(Constants.DriveConstants.ConfigFolder);
  }

  public DriveSubsystem(String configFolder) {

    File directory = new File(Filesystem.getDeployDirectory(), configFolder);
    try {
      System.out.println("loading SwerveDrive: " + directory);
      m_SwerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.DriveConstants.kMaxVelocityMetersPerSec);
    } catch (Exception e) {
      System.out.println("Swerve Configuration failed! " + e); // todo throw a fatal exception here?
    }

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveRobotOriented(Double x, Double y, Double turn) {
    Translation2d translation = new Translation2d(x, y);
    translation = translation.times(m_SwerveDrive.getMaximumChassisVelocity());
    turn *= m_SwerveDrive.getMaximumChassisAngularVelocity();
    m_SwerveDrive.drive(new ChassisSpeeds(translation.getX(), translation.getY(), turn));
  }

  public void driveFieldOriented(Double x, Double y, Double turn) {
    Translation2d translation = new Translation2d(x, y);
    translation = translation.times(m_SwerveDrive.getMaximumChassisVelocity());
    turn *= m_SwerveDrive.getMaximumChassisAngularVelocity();
    m_SwerveDrive.driveFieldOriented(new ChassisSpeeds(translation.getX(), translation.getY(), turn));
  }

  public double getGyroAngleDegrees() {
    return m_SwerveDrive.getYaw().getDegrees();
  }

  public void setGyroAngleDeg(double angle) {
    m_SwerveDrive.setGyro(new Rotation3d(0, 0, Math.toRadians(angle)));
  }

  public double getGyroAngleRadians() {
    return m_SwerveDrive.getYaw().getRadians();
  }

  public double getTurnRate() {
    return m_SwerveDrive.getGyro().getYawAngularVelocity().in(Units.DegreesPerSecond);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_SwerveDrive.getRobotVelocity();
  }

  public double getMaxSpeedLimit() {
    return m_SwerveDrive.getMaximumChassisVelocity();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return m_SwerveDrive.getModulePositions();
  }

  public SwerveDriveKinematics getKinematics() {
    return m_SwerveDrive.kinematics;
  }

  public double turnToHeadingDegrees(double heading) {
    double currentRad = getGyroAngleRadians();
    double targetRad = Math.toRadians(heading);
    return Math.toDegrees(m_SwerveDrive.getSwerveController().headingCalculate(currentRad, targetRad));
  }

  public void drive(ChassisSpeeds speeds, Translation2d centerOfRotation) {
    m_SwerveDrive.drive(speeds, false, centerOfRotation);
  }

  public void drive(ChassisSpeeds speeds) {
    drive(speeds, Translation2d.kZero);
  }

  public void testSetAll(double voltage, double angleRadians) {
    SwerveModuleState state = new SwerveModuleState(voltage, Rotation2d.fromRadians(angleRadians));
    SwerveModule[] modules = m_SwerveDrive.getModules();
    for (SwerveModule mod : modules) {
      mod.setDesiredState(state, false, false);
    }
  }

  public void stop() {
    drive(new ChassisSpeeds(0, 0, 0));
  };

  public void driveHeadingField(Translation2d translationMetersPerSecond, double heading) {
    double yawCommand = turnToHeadingDegrees(heading);
    driveFieldOriented(translationMetersPerSecond, yawCommand);
  }

  public void driveHeadingRobot(Translation2d translationMetersPerSecond, double heading) {
    double yawCommand = turnToHeadingDegrees(heading);
    driveRobotOriented(translationMetersPerSecond, yawCommand);
  }

  public void driveFieldOriented(ChassisSpeeds speeds) {
    ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        Rotation2d.fromDegrees(getGyroAngleDegrees()));

    drive(robotSpeeds);
  }

  public void driveFieldOriented(Translation2d translation, double rotation) {
    ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    driveFieldOriented(speeds);
  }

  public void driveRobotOriented(Translation2d translation, double rotation) {
    ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    drive(speeds);
  }

  public double getSpeed() {
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
  }

  /*
   * Returns the velocity vector of the robot, in the Robot Frame, in meters per
   * second.
   */
  public Translation2d getVelocityVector() {
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
  }

  public void orbitRobotFrame(double orbitSpeed, Translation2d center) {
    drive(new ChassisSpeeds(0, 0, orbitSpeed), center);
  }

  public PIDController getYawPID(){
    return m_SwerveDrive.getSwerveController().thetaController;
  }
}
