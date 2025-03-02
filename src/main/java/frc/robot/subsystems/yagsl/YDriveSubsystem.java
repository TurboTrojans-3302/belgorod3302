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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // todo add the swerve drive to the dashboard
    SwerveModule[] module = m_SwerveDrive.getModules();
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> Math.toRadians(module[0].getAbsolutePosition()), null);
        builder.addDoubleProperty("Front Left Velocity", () -> module[0].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> Math.toRadians(module[1].getAbsolutePosition()), null);
        builder.addDoubleProperty("Front Right Velocity", () -> module[1].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> Math.toRadians(module[2].getAbsolutePosition()), null);
        builder.addDoubleProperty("Back Left Velocity", () -> module[2].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> Math.toRadians(module[3].getAbsolutePosition()), null);
        builder.addDoubleProperty("Back Right Velocity", () -> module[3].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> getGyroAngleRadians(), null);
      }
    });

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

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("gyroAngleDegrees", this::getGyroAngleDegrees, this::setGyroAngleDeg);
      builder.addDoubleProperty("MaxSpeedLimit", () -> {
          return m_SwerveDrive.getMaximumChassisAngularVelocity();
      }, (x) -> {
          m_SwerveDrive.setMaximumAllowableSpeeds(x, m_SwerveDrive.getMaximumChassisAngularVelocity());
      });
      builder.addDoubleProperty("MaxRotationLimit", () -> {
          return m_SwerveDrive.getMaximumChassisAngularVelocity();
      }, (x) -> {
          m_SwerveDrive.setMaximumAllowableSpeeds(getMaxSpeed(), x);
      });
      builder.addStringProperty("CommandedSpeeds",
      ()->{ChassisSpeeds CommandedSpeeds = m_SwerveDrive.getRobotVelocity();
           return String.format("x:%5.2f y:%5.2f (%5.2f %5.2f deg) %4.2f",
                        CommandedSpeeds.vxMetersPerSecond,
                        CommandedSpeeds.vyMetersPerSecond,
                        Math.hypot(CommandedSpeeds.vxMetersPerSecond, CommandedSpeeds.vyMetersPerSecond),
                        Math.toDegrees(Math.atan2(CommandedSpeeds.vxMetersPerSecond, CommandedSpeeds.vyMetersPerSecond)),
                        CommandedSpeeds.omegaRadiansPerSecond
                        );},
      null
      );

  }

}
