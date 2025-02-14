// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.ExtendGripper;
import frc.robot.commands.MoveRobotAndElevator;
import frc.robot.commands.OpenGripper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Navigation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReefSideTestL1Red extends SequentialCommandGroup {
  /** Creates a new ReefSideCenterScore. */
  DriveSubsystem m_drive;
  Navigation m_nav;
  Gripper m_gripper;
  Elevator m_elevator;
  int aprilTag = 10;
  Pose2d aprilTagPose;
  public ReefSideTestL1Red(DriveSubsystem drive, Navigation nav, Gripper gripper, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drive = drive;
    m_nav = nav;
    m_gripper = gripper;
    m_elevator = elevator;
    aprilTagPose = m_nav.getPose2dInFrontOfTag(aprilTag, 1.0);

    
    addCommands(new MoveRobotAndElevator(m_drive, m_nav, m_elevator, aprilTagPose, Constants.ElevatorConstants.kLevel1Trough),
                //gripper closing runs at the same time as the main command
                new DriveToAprilTag(drive, nav, aprilTag),
                //keep gripper closed runs at the same time as the main command
                new ExtendGripper(m_gripper),
                //some time for the gripper extension to settle before dropping the coral
                new OpenGripper(m_gripper));
  }
}
