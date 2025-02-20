// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Ludwig.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToCoral extends SequentialCommandGroup {
  /** Creates a new CoralChaserTeleop. */
  DriveSubsystem m_drive;
  Navigation m_nav;
  boolean rightOnly;
  boolean leftOnly;
  boolean auto;
  double maxSpeed;
  double maxAcceleration;
  double maxAngularSpeedRadians;
  double angleTolerance;
  public DriveToCoral(DriveSubsystem drive, Navigation nav, double targetRange, boolean limitedToRightSide, boolean limitedToLeftSide, double directionalAngleTolerance, boolean autonRunning) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_nav = nav;
    m_drive = drive;
    rightOnly = limitedToRightSide;
    leftOnly = limitedToLeftSide;
    auto = autonRunning;
    angleTolerance = directionalAngleTolerance; //can be zero if you want to exclude all objects on one side of the camera image

    if (auto){
      maxSpeed = Constants.AutoConstants.kMaxSpeedMetersPerSecond;
      maxAcceleration = Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
      maxAngularSpeedRadians = Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;
    } else {
      maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
      maxAcceleration = AutoConstants.kMaxAccelerationMetersPerSecondSquared; //TODO there isn't a constant for this outside of autonomous so what should it be?
      maxAngularSpeedRadians = DriveConstants.kMaxAngularSpeed;
    }
    addCommands(new ChangeLimelightPipeline(m_nav, Constants.LimelightConstants.PipelineIdx.NeuralClassifer),
    new CoralDistanceAlgorithm(m_nav, rightOnly, leftOnly, angleTolerance, maxSpeed, maxAcceleration, maxAngularSpeedRadians),
    GoToCommand.relative(m_drive, m_nav, 0, 0, m_nav.getTargetTX()),
    GoToCommand.relative(m_drive, m_nav, m_nav.getTargetDistanceFiducial() + targetRange, 0, 0));
  }
}
