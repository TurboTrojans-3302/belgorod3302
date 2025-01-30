// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToAprilTag extends Command {

  DriveSubsystem m_drive;
  int m_apriltag;
  AprilTagFinder m_finder;
  double distance;

  /** Creates a new TurnToAprilTag. */
  public TurnToAprilTag(DriveSubsystem drive, int apriltag, AprilTagFinder finder) {
    m_drive = drive;
    m_apriltag = apriltag;
    m_finder = finder;
    distance = m_finder.getDistanceToTarget();
    
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_finder.setTarget(m_apriltag);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Double heading = m_drive.getAngleDeg();

    if(m_finder.isTargetFound()) {
      heading = heading - m_finder.getAngleToTarget();
      m_drive.driveHeading(new Translation2d(0.0, 0.0), (heading / 5));
    }

    SmartDashboard.putNumber("Heading", heading);
    SmartDashboard.putNumber("Target Angle", m_finder.getAngleToTarget());
    SmartDashboard.putBoolean("Target Found", m_finder.isTargetFound());
    distance = m_finder.getDistanceToTarget();
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
