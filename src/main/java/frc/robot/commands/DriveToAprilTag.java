// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.Pose3dMedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTag extends Command {
  public static final String cameraName = "limelight";

  DriveSubsystem m_drive;
  int m_targetTag;
  boolean targetFound = false;
  Pose3d targetPose;
  Pose3d startPose;
  Pose3dMedianFilter filter = new Pose3dMedianFilter(100);

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
    startPose = new Pose3d(m_drive.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (LimelightHelpers.getTV(cameraName)) {
      Pose3d targetRelativePose = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName);
      Transform3d delta = new Transform3d(startPose, targetRelativePose);
      Pose3d targetAbsolutePose = targetRelativePose.plus(delta);
      targetPose = filter.calculate(targetAbsolutePose);
      targetFound = true;
    }

    Double heading = m_drive.getHeading();
    double ySpeedRobot = 0.0;

    if (targetFound) {
      Double errorAngle = getAngleToTarget();
      heading = heading + (errorAngle * TurnFactor);

      double bearingAngle = Math.toDegrees(getYdistance());
      ySpeedRobot = bearingAngle * kPyaxis;
    }

    m_drive.driveHeadingRobot(new Translation2d(ForwardSpeed, ySpeedRobot), heading);

    SmartDashboard.putNumber("Target Angle", getAngleToTarget());
    SmartDashboard.putNumber("Commanded heading", heading);
    SmartDashboard.putNumber("Target Distance", getDistance());
    SmartDashboard.putNumber("Y error", getYdistance());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getDistanceToObjectMeters() < TargetDistance;
  }

  public Transform3d getTransform3dToTarget() {
    Pose3d botPose = new Pose3d(m_drive.getPose());
    Transform3d delta = new Transform3d(botPose, targetPose);
    return delta;
  }

  public Translation2d getTranslation2dToTarget() {
    return getTransform3dToTarget().getTranslation().toTranslation2d();
  }
  public Double getAngleToTarget() {
    return getTranslation2dToTarget().getAngle().getDegrees();
  }

  public double getDistance() {
    return getTranslation2dToTarget().getNorm();
  }

  public double getYdistance() {
    Pose2d botPose3d = m_drive.getPose();
    Pose2d relativeBotPose = botPose3d.relativeTo(targetPose.toPose2d());
    return relativeBotPose.getY();
  }
}
