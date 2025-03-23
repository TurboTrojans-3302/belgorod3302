// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCloseToReef;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.OrbitReefToTag;
import frc.robot.commands.TroughScore;
import frc.robot.subsystems.Navigation;

public class AutonCoralTrough extends SequentialCommandGroup {
  public AutonCoralTrough(RobotContainer bot, int tagid) {
            addCommands(new DriveCloseToReef(bot.m_robotDrive, bot.m_nav),

                       new OrbitReefToTag(bot.m_robotDrive, bot.m_nav, tagid),
                                       
                       new GoToCommand(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(tagid, 0.05)),
                       new TroughScore(bot.m_intakeArm, bot.m_intake),
                       GoToCommand.relative(bot.m_robotDrive, bot.m_nav, -1.0, 0, 0)
                      );
  }
}
