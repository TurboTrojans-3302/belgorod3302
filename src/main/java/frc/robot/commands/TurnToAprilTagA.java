// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/*
 * TODO Saturday
 * 
 * Find a limelight camera - done
 * Mount it on the robot (at the real height) - kinda done
 * 
 * Limelight can see the apriltag form ~12' away
 * 
 * Dial in the TurnFactor constant
 * Question: Can we see the apriltag while driving?
 * 
 * 
 * 
 */


 /* RoboRIO and webcam has maximum range of ~57inches */

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToAprilTagA extends Command {

  DriveSubsystem m_drive;
  int m_targetTag;
  boolean targetFound = false;
  RawFiducial detectedTarget;

  public static final double TurnTolerance = 1.5;
  public static final double TurnFactor = .2;

  /** Creates a new TurnToAprilTag. */
  public TurnToAprilTagA(DriveSubsystem drive, int apriltag) {
    m_drive = drive;
    m_targetTag = apriltag;
    
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    lookForTarget();

    Double heading = m_drive.getAngleDeg();

    if(isTargetFound()) {
        Double errorAngle = getAngleToTarget();
        heading = heading  - (errorAngle * TurnFactor);
        SmartDashboard.putNumber("Target Angle", getAngleToTarget());
    }

    m_drive.driveHeading(new Translation2d(0.0, 0.0), heading);

    SmartDashboard.putNumber("Heading", heading);
    SmartDashboard.putBoolean("Target Found", isTargetFound());
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }

  private void lookForTarget() {
    targetFound = false;
    detectedTarget = null;

    RawFiducial[] detectedTags = LimelightHelpers.getRawFiducials("limelight");

    for(RawFiducial tag : detectedTags) {
      if(tag.id == m_targetTag) {
        targetFound = true;
        detectedTarget = tag;
      }
    }
  }


  boolean isTargetFound() {
    return targetFound;
  }

  Double getAngleToTarget() {
    Double angle = 0.0;

    if(isTargetFound()) {
      angle = detectedTarget.txnc;
    } 

    return angle;
  }
}
