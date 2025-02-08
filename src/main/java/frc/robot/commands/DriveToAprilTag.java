// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTag extends Command {
  public static final String cameraName = "limelight";

  DriveSubsystem m_drive;
  int m_targetTag;
  boolean targetFound = false;

  RawFiducial detectedTarget;
  double ySpeedRobot = 0.0;

  public static final double TurnTolerance = 1.5;
  public static final double TurnFactor = 0.5;
  public static final double ForwardSpeed = 0.2;
  public static final double kPyaxis = 0.1;
  public static final double TargetDistance = 0.09; // meters
  public static final double alignmentTolerance = 0.25; // for x-direction camera pose in relation to target

  /** Creates a new TurnToAprilTag. */
  public DriveToAprilTag(DriveSubsystem drive, int apriltag) {
    m_drive = drive;
    m_targetTag = apriltag;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPriorityTagID(cameraName, m_targetTag);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Double heading = m_drive.getHeading();

    if (LimelightHelpers.getTV(cameraName)) {
      Double errorAngle = getAngleToTarget();
      heading = heading + (errorAngle * TurnFactor);

    }
if (isTargetFound()){
  double bearingAngle = Math.toDegrees(getZradians());
    ySpeedRobot = bearingAngle * kPyaxis;
  }
    
    
    // camera coordinates use X for left and right, unlike the translation which
    // uses Y
    /*
     * if (bearingAngle > alignmentTolerance){
     * //if camera is to the right of target
     * ySpeedRobot = -SidewaysSpeed;
     * } else if (bearingAngle < -alignmentTolerance){
     * //to the left
     * ySpeedRobot = SidewaysSpeed;
     * } else {
     * //within tolerance
     * ySpeedRobot = 0.0;
     * 
     */
 

    m_drive.driveHeadingRobot(new Translation2d(ForwardSpeed, ySpeedRobot), heading);
    SmartDashboard.putNumber("Target Angle", getAngleToTarget());
    SmartDashboard.putNumber("Commanded heading", heading);
    SmartDashboard.putNumber("Target Distance", getDistance());
    SmartDashboard.putNumber("degrees Z", (getZradians() * 360.0) / 6.28);
    SmartDashboard.putBoolean("Target Found", isTargetFound());
    // SmartDashboard.putBoolean("valid pose est",
    // LimelightHelpers.validPoseEstimate(null))

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((m_drive.getDistanceToObjectMeters() < TargetDistance)) {
      return true;
    } else {
      return false;
    }

  }

  boolean isTargetFound() {
    return LimelightHelpers.getTV(cameraName);
  }

  static public Double getAngleToTarget() {
    // Translation2d target =
    // LimelightHelpers.getTargetPose3d_RobotSpace(cameraName)
    // .getTranslation()
    // .toTranslation2d();
    // return target.getAngle().getDegrees();
    return -LimelightHelpers.getTX(cameraName);
  }

  public double getDistance() {
    Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(cameraName);
    double distance = targetPose.getTranslation().getNorm();
    System.out.println("Distance:" + distance);

    return distance;
  }

  // positive x in a camera pose means camera is to the right of the tag
  // todo configure the camera space offset
  static public double getZradians() {
    Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName);
    double rad = targetPose.getRotation().getZ();
    return -rad;
  }
}
