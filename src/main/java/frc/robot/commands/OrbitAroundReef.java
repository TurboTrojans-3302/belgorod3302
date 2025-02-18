// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.frc2025.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OrbitAroundReef extends Command {
  private DriveSubsystem drive;
  private Navigation nav;
  private double speed;
  
    /** Creates a new OrbitAroundReef. */
    public OrbitAroundReef(DriveSubsystem drive, Navigation nav, double speed) {
      this.drive = drive;
      this.nav = nav;
      this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d reef = FieldConstants.Reef.center;
    Pose2d robot = nav.getPose();
    Translation2d robotTranslationType = robot.getTranslation();
    Translation2d C = reef.minus(robotTranslationType);
    Rotation2d angleOfVector = C.getAngle();
    Translation2d finalVector = new Translation2d(speed, angleOfVector.plus(Rotation2d.fromDegrees(90)));
    drive.driveFieldOriented(finalVector, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
