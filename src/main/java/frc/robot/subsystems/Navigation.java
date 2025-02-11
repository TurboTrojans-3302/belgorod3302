// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Navigation extends SubsystemBase {
  private static final String cameraName = "limelight";

  private DriveSubsystem m_drive;
  public Field2d m_dashboardField = new Field2d();
  private AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  protected SwerveDrivePoseEstimator m_odometry;
  private LaserCan m_dxSensor = new LaserCan(Constants.DX_SENSOR_CAN_ID);

  /** Creates a new Navigation. */
  public Navigation(DriveSubsystem drive) {
    this.m_drive = drive;

    m_odometry = new SwerveDrivePoseEstimator(
        m_drive.getKinematics(), Rotation2d.fromDegrees(m_drive.getGyroAngleDegrees()),
        m_drive.getSwerveModulePositions(),
        Constants.FieldConstants.ZeroZero);

    try {
      m_dxSensor.setRangingMode(LaserCan.RangingMode.LONG);
      m_dxSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      m_dxSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("LaserCan Configuration failed! " + e);
    }

    SmartDashboard.putData(m_dashboardField);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SwerveModulePosition[] positions = m_drive.getSwerveModulePositions();
    Double heading = m_drive.getGyroAngleDegrees();
    m_odometry.update(Rotation2d.fromDegrees(heading), positions);

    PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    // todo is this necessary? how often is the estimate invalid?
    if (LimelightHelpers.validPoseEstimate(est)) {
      m_odometry.addVisionMeasurement(est.pose, est.timestampSeconds);
    }

    m_dashboardField.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(Rotation2d.fromDegrees(m_drive.getGyroAngleDegrees()),
        m_drive.getSwerveModulePositions(),
        pose);
  }

  public Double getDxToObjectMeters() {
    Measurement m = m_dxSensor.getMeasurement();
    return m.distance_mm * 0.001;
  }

  public boolean dxMeasurmentGood() {
    Measurement m = m_dxSensor.getMeasurement();
    return m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
  }

  public Pose2d getTagPose2d(int tagId) {
    return m_fieldLayout.getTagPose(tagId).get().toPose2d();
  }

  public Pose2d getPose2dInFrontOfTag(int tagId, double distance) {
    Transform2d delta = new Transform2d(distance, 0.0, Rotation2d.fromDegrees(180.0));
    Pose2d tagPose = getTagPose2d(tagId);
    return tagPose.plus(delta);
  }


  /**
   * @return heading angle of the bot, according to the odometry, in degrees
   */
  public double getAngleDegrees() {
    return m_odometry.getEstimatedPosition().getRotation().getDegrees();
  }

}
