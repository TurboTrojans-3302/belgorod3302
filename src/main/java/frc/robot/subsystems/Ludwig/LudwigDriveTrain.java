// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Ludwig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16448_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystemBase;

public class LudwigDriveTrain extends DriveSubsystemBase {
  private double maxSpeedLimit = DriveConstants.kMaxSpeedMetersPerSecond; // m/s
  private double maxRotationLimit = DriveConstants.kMaxAngularSpeed;


  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = MAXSwerveModule.getInstance(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  public MAXSwerveModule getM_frontLeft() {
    return m_frontLeft;
  }

  private final MAXSwerveModule m_frontRight = MAXSwerveModule.getInstance(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  public MAXSwerveModule getM_frontRight() {
    return m_frontRight;
  }

  private final MAXSwerveModule m_rearLeft = MAXSwerveModule.getInstance(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  public MAXSwerveModule getM_rearLeft() {
    return m_rearLeft;
  }

  private final MAXSwerveModule m_rearRight = MAXSwerveModule.getInstance(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  public MAXSwerveModule getM_rearRight() {
    return m_rearRight;
  }

  // The gyro sensor
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU(IMUAxis.kX, SPI.Port.kMXP, CalibrationTime._1s);
  private double m_gyroOffsetDeg = 0.0;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;

  public double getM_currentTranslationDir() {
    return m_currentTranslationDir;
  }

  private double m_currentTranslationMag = 0.0;

  public double getM_currentTranslationMag() {
    return m_currentTranslationMag;
  }

  private PIDController headingPidController;

  /** Creates a new DriveSubsystem. */
  public LudwigDriveTrain() {
    headingPidController = new PIDController(DriveConstants.headingP,
        DriveConstants.headingI,
        DriveConstants.headingD);
    headingPidController.enableContinuousInput(0.0, 360.0);
    headingPidController.setTolerance(2.0);
  
      // todo add the swerve drive to the dashboard
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getSteerAngle(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getDriveVelocity(), null);

                builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getSteerAngle(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getDriveVelocity(), null);

                builder.addDoubleProperty("Back Left Angle", () -> m_rearLeft.getSteerAngle(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> m_rearLeft.getDriveVelocity(), null);

                builder.addDoubleProperty("Back Right Angle", () -> m_rearRight.getSteerAngle(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> m_rearRight.getDriveVelocity(), null);

                builder.addDoubleProperty("Robot Angle", () -> getGyroAngleRadians(), null);
            }
        });
  }

  @Override
  public void periodic() {
    setMaxSpeed();

  }

  public double turnToHeadingDegrees(double targetHeading) {
    double currentHeading = getGyroAngleDegrees();
    double rotation = headingPidController.calculate(currentHeading, targetHeading);
    return rotation;
  }

  public void driveRobotOriented(Double xSpeed, Double ySpeed, Double rot) {

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * maxSpeedLimit;
    double ySpeedDelivered = ySpeed * maxSpeedLimit;
    double rotDelivered = m_currentRotation * maxRotationLimit;

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    drive(speeds);
  }

  public void driveFieldOriented(Double xSpeed, Double ySpeed, Double rot) {

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * maxSpeedLimit;
    double ySpeedDelivered = ySpeed * maxSpeedLimit;
    double rotDelivered = m_currentRotation * maxRotationLimit;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
        Rotation2d.fromDegrees(getGyroAngleDegrees()));

    drive(speeds);
  }

  public void drive(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeedLimit);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void testSetAll(double voltage, double angle) {
    m_frontLeft.testSet(voltage, angle);
    m_frontRight.testSet(voltage, angle);
    m_rearLeft.testSet(voltage, angle);
    m_rearRight.testSet(voltage, angle);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void setGyroAngleDeg(double angle) {
    m_gyroOffsetDeg = angle - m_gyro.getAngle();
  }

  public double getGyroAngleRadians() {
    return MathUtil.angleModulus(Units.degreesToRadians(getGyroAngleDegrees()));
  }

  public double getGyroAngleDegrees(){
    return m_gyro.getAngle() + m_gyroOffsetDeg;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public void setP(double val) {
    headingPidController.setP(val);
  }

  public void setI(double val) {
    headingPidController.setI(val);
  }

  public void setD(double val) {
    headingPidController.setD(val);
  }

  @Override
  public double getMaxSpeedLimit() {
    return DriveConstants.kMaxSpeedMetersPerSecond;
  }

  @Override
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] { m_frontLeft.getPosition(),
                                        m_frontRight.getPosition(),
                                        m_rearLeft.getPosition(),
                                        m_rearRight.getPosition()
                                      };
  };

  @Override
  public SwerveDriveKinematics getKinematics() {
    return DriveConstants.kinematics;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("gyroAngleDegrees", this::getGyroAngleDegrees, this::setGyroAngleDeg);
      builder.addDoubleProperty("MaxSpeedLimit", () -> {
          return maxSpeedLimit;
      }, (x) -> {
          maxSpeedLimit = x;
      });
      builder.addDoubleProperty("MaxRotationLimit", () -> {
          return maxRotationLimit;
      }, (x) -> {
          maxRotationLimit = x;
      });
  }}
