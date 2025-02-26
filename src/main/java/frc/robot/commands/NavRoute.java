// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NavRoute extends GoToCommand {
  private List<Pose2d> waypoints;
  private int index;

  /** Creates a new NavRoute. */
  public NavRoute(DriveSubsystem drive, Navigation nav, List<Pose2d> waypoints) {
    super(drive, nav);
    this.waypoints = waypoints;
    m_dest = waypoints.get(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d bot = m_nav.getPose();
    Pose2d nearest = bot.nearest(waypoints);
    index = waypoints.indexOf(nearest);
    m_dest = waypoints.get(index);
    super.initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(super.isFinished()) {
      index++;
      if(index == waypoints.size()){
        return true;
      }
      m_dest = waypoints.get(index);
      super.initialize();
    }

    return false;
  }
}
