// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yagsl;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystemBase;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class YDriveSubsystem extends DriveSubsystemBase {
  /** Creates a new YDriveSubsystem. */

  private SwerveDrive m_SwerveDrive;

  public YDriveSubsystem(){
    this(Constants.DriveConstants.ConfigFolder);
  }

  public YDriveSubsystem(String configFolder) {

    File directory = new File(Filesystem.getDeployDirectory(), configFolder);
    try {
      System.out.println("loading SwerveDrive: " + directory);
      m_SwerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.DriveConstants.kMaxVelocityMetersPerSec);
    } catch (Exception e) {
      System.out.println("Swerve Configuration failed! " + e); //todo throw a fatal exception here?
    }

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void driveRobotOriented(Double x, Double y, Double turn) {
    Translation2d translation = new Translation2d(x, y);
    translation = translation.times(m_SwerveDrive.getMaximumChassisVelocity());
    turn *= m_SwerveDrive.getMaximumChassisAngularVelocity();
    m_SwerveDrive.drive(new ChassisSpeeds(translation.getX(), translation.getY(), turn));
  }

  @Override
  public void driveFieldOriented(Double x, Double y, Double turn) {
    Translation2d translation = new Translation2d(x, y);
    translation = translation.times(m_SwerveDrive.getMaximumChassisVelocity());
    turn *= m_SwerveDrive.getMaximumChassisAngularVelocity();
    m_SwerveDrive.driveFieldOriented(new ChassisSpeeds(translation.getX(), translation.getY(), turn));
  }

  @Override
  public double getGyroAngleDegrees() {
    return m_SwerveDrive.getYaw().getDegrees();
  }

  @Override
  public void setGyroAngleDeg(double angle) {
    m_SwerveDrive.setGyro(new Rotation3d(0, 0, Math.toRadians(angle)));
  }

  @Override
  public double getGyroAngleRadians() {
    return m_SwerveDrive.getYaw().getRadians();
  }

  @Override
  public double getTurnRate() {
    return m_SwerveDrive.getGyro().getYawAngularVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return m_SwerveDrive.getRobotVelocity();
  }

  @Override
  public double getMaxSpeedLimit() {
    return m_SwerveDrive.getMaximumChassisVelocity();
  }

  @Override
  public SwerveModulePosition[] getSwerveModulePositions() {
    return m_SwerveDrive.getModulePositions();
  }

  @Override
  public SwerveDriveKinematics getKinematics() {
    return m_SwerveDrive.kinematics;
  }

  @Override
  public double turnToHeadingDegrees(double heading) {
    double currentRad = getGyroAngleRadians();
    double targetRad = Math.toRadians(heading); 
    return Math.toDegrees(m_SwerveDrive.getSwerveController().headingCalculate(currentRad, targetRad));
  }

  @Override
  public void drive(ChassisSpeeds speeds, Translation2d centerOfRotation) {
    m_SwerveDrive.drive(speeds, false, centerOfRotation);
  }

  @Override
  public void testSetAll(double voltage, double angleRadians) {
    SwerveModuleState state = new SwerveModuleState(voltage, Rotation2d.fromRadians(angleRadians));
    SwerveModule[] modules = m_SwerveDrive.getModules();
    for (SwerveModule mod : modules) {
      mod.setDesiredState(state, false, false);
    }
  }
}
