// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.List;

import org.littletonrobotics.frc2025.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.GoAlmostTo;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoNearestIceCream extends GoAlmostTo {
  private final List<Pose2d> targetList = List.of(
      FieldConstants.StagingPositions.leftIceCream,
      FieldConstants.StagingPositions.middleIceCream,
      FieldConstants.StagingPositions.rightIceCream,
      FieldConstants.redVersion(FieldConstants.StagingPositions.leftIceCream),
      FieldConstants.redVersion(FieldConstants.StagingPositions.middleIceCream),
      FieldConstants.redVersion(FieldConstants.StagingPositions.rightIceCream)
      );

  /** Creates a new GoNearestIceCream. */
  public GoNearestIceCream(DriveSubsystem drive, Navigation nav) {
    super(drive, nav, null, 0.6);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dest = m_nav.getPose().nearest(targetList);
    super.initialize();
  }
}
