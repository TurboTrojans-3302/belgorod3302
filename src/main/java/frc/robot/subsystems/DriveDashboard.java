// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveDashboard extends SubsystemBase {
  private DriveSubsystem m_drive;
  private ShuffleboardTab m_shuffleboardTab;
  private GenericEntry m_FLangleEntry, m_FRangleEntry, m_BLangleEntry, m_BRangleEntry;
  private GenericEntry m_FLspeedEntry, m_FRspeedEntry, m_BLspeedEntry, m_BRspeedEntry;
  private GenericEntry m_gyroEntry, m_driveDirEntry, m_driveMagEntry;
  private GenericEntry m_fieldOrientedEntry, m_xPosEntry, m_yPosEntry;
  private GenericEntry mPEntry, mIEntry, mDEntry;

  /** Creates a new DriveDashboard. */
  public DriveDashboard(DriveSubsystem drive) {
    m_drive = drive;

    m_shuffleboardTab = Shuffleboard.getTab("Drive");
    m_FLangleEntry = m_shuffleboardTab.add("FLa", 0.0)
        .withWidget(BuiltInWidgets.kGyro)
        .withProperties(Map.of("Counter clockwise", true))
        .withPosition(0, 0).withSize(3, 3)
        .getEntry();
    m_FRangleEntry = m_shuffleboardTab.add("FRa", 0.0)
        .withWidget(BuiltInWidgets.kGyro)
        .withProperties(Map.of("Counter clockwise", true))
        .withPosition(4, 0).withSize(3, 3)
        .getEntry();
    m_BLangleEntry = m_shuffleboardTab.add("BLa", 0.0)
        .withWidget(BuiltInWidgets.kGyro)
        .withProperties(Map.of("Counter clockwise", true))
        .withPosition(0, 3).withSize(3, 3)
        .getEntry();
    m_BRangleEntry = m_shuffleboardTab.add("BRa", 0.0)
        .withWidget(BuiltInWidgets.kGyro)
        .withProperties(Map.of("Counter clockwise", true))
        .withPosition(4, 3).withSize(3, 3)
        .getEntry();
    m_FLspeedEntry = m_shuffleboardTab.add("FLs", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("orientation", "vertical",
            "min", -DriveConstants.kMaxSpeedMetersPerSecond,
            "max", DriveConstants.kMaxSpeedMetersPerSecond))
        .withPosition(3, 0).withSize(1, 3)
        .getEntry();
    m_FRspeedEntry = m_shuffleboardTab.add("FRs", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("orientation", "vertical",
            "min", -DriveConstants.kMaxSpeedMetersPerSecond,
            "max", DriveConstants.kMaxSpeedMetersPerSecond))
        .withPosition(7, 0).withSize(1, 3)
        .getEntry();
    m_BLspeedEntry = m_shuffleboardTab.add("BLs", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("orientation", "vertical",
            "min", -DriveConstants.kMaxSpeedMetersPerSecond,
            "max", DriveConstants.kMaxSpeedMetersPerSecond))
        .withPosition(3, 3).withSize(1, 3)
        .getEntry();
    m_BRspeedEntry = m_shuffleboardTab.add("BRs", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("orientation", "vertical",
            "min", -DriveConstants.kMaxSpeedMetersPerSecond,
            "max", DriveConstants.kMaxSpeedMetersPerSecond))
        .withPosition(7, 3).withSize(1, 3)
        .getEntry();
    m_gyroEntry = m_shuffleboardTab.add("Gyro", 0.0)
        .withWidget(BuiltInWidgets.kGyro)
        .withProperties(Map.of("Counter clockwise", true))
        .withPosition(9, 0).withSize(3, 3)
        .getEntry();
    m_driveDirEntry = m_shuffleboardTab.add("Drive Dir", 0.0)
        .withWidget(BuiltInWidgets.kGyro)
        .withProperties(Map.of("Counter clockwise", true))
        .withPosition(12, 0).withSize(3, 3)
        .getEntry();
    m_driveMagEntry = m_shuffleboardTab.add("Drive Mag", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("orientation", "vertical",
            "min", -DriveConstants.kMaxSpeedMetersPerSecond,
            "max", DriveConstants.kMaxSpeedMetersPerSecond))
        .withPosition(15, 0).withSize(1, 3)
        .getEntry();
    m_fieldOrientedEntry = m_shuffleboardTab.add("Field Oriented", true)
                                        .withWidget(BuiltInWidgets.kToggleSwitch)
                                        .getEntry();
    m_xPosEntry = m_shuffleboardTab.add("x position", 0.0)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
    m_yPosEntry = m_shuffleboardTab.add("y position", 0.0)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
    mPEntry = m_shuffleboardTab.add("P", Constants.DriveConstants.headingP)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withPosition(7, 0).withSize(1, 1)
                        .getEntry();
    mIEntry = m_shuffleboardTab.add("I", Constants.DriveConstants.headingI)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withPosition(7, 1).withSize(1, 1)
                        .getEntry();
    mDEntry = m_shuffleboardTab.add("D", Constants.DriveConstants.headingD)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withPosition(7, 2).withSize(1, 1)
                        .getEntry();
  }

  @Override
  public void periodic() {
    m_FLangleEntry.setDouble(m_drive.getM_frontLeft().getPosition().angle.getDegrees());
    m_FRangleEntry.setDouble(m_drive.getM_frontRight().getPosition().angle.getDegrees());
    m_BRangleEntry.setDouble(m_drive.getM_rearRight().getPosition().angle.getDegrees());
    m_BLangleEntry.setDouble(m_drive.getM_rearLeft().getPosition().angle.getDegrees());
    m_FLspeedEntry.setDouble(m_drive.getM_frontLeft().getState().speedMetersPerSecond);
    m_FRspeedEntry.setDouble(m_drive.getM_frontRight().getState().speedMetersPerSecond);
    m_BRspeedEntry.setDouble(m_drive.getM_rearRight().getState().speedMetersPerSecond);
    m_BLspeedEntry.setDouble(m_drive.getM_rearLeft().getState().speedMetersPerSecond);

    m_driveDirEntry.setDouble(m_drive.getM_currentTranslationDir());
    m_driveMagEntry.setDouble(m_drive.getM_currentTranslationMag());

    m_gyroEntry.setDouble(m_drive.getHeading());
    Pose2d pos = m_drive.getPose();
    m_xPosEntry.setDouble(pos.getX());
    m_yPosEntry.setDouble(pos.getY());

    m_drive.setP(mPEntry.getDouble(Constants.DriveConstants.headingP));
    m_drive.setI(mIEntry.getDouble(Constants.DriveConstants.headingI));
    m_drive.setD(mDEntry.getDouble(Constants.DriveConstants.headingD));
  }

  public boolean getFieldOriented(){
    return m_fieldOrientedEntry.getBoolean(true);
  }
}
