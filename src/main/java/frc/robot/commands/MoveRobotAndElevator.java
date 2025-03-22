// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Navigation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveRobotAndElevator extends ParallelCommandGroup {
  /** Creates a new MoveRobotAndElevator. */
  DriveSubsystem m_drive;
  Elevator m_elevator;
  Navigation m_nav;
  Pose2d commandedPose;
  double elevatorPosition;
  public MoveRobotAndElevator(DriveSubsystem drive, Navigation nav, Elevator elevator, Pose2d pose, double elevatorSetpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drive = drive;
    m_nav = nav;
    m_elevator = elevator;
    elevatorPosition = elevatorSetpoint;
    commandedPose = pose;
    // or do I need to create a new parallel command instance instead?
    addCommands(new GoToCommand(drive, nav, commandedPose),
                new MoveElevator(elevator, elevatorPosition)
               );
  }
}
