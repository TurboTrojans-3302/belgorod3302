// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonRemoveAlgae extends SequentialCommandGroup {
  /** Creates a new AutonRemoveAlgae. */

  public AutonRemoveAlgae(RobotContainer bot, int tagID) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(new DriveCloseToReef(bot.m_robotDrive, bot.m_nav),

                       new OrbitReefToTag(bot.m_robotDrive, bot.m_nav, tagID),
                                       
                       new GoToCommand(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(tagID, 0.457))
                                      .withTimeout(8.0),
                       bot.m_elevator.algaeLevelCommand(),

                       bot.m_gripper.extendCommand(),

                       bot.m_elevator.level4Command()
                       .alongWith(GoToCommand.relative(bot.m_robotDrive, bot.m_nav, -1.0, 0.0, 0.0)),

                      bot.m_gripper.retractCommand(),
                      bot.m_elevator.loadPosCommand()
                     

                                      );
  }

  
}
