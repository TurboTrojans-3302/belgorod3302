// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoCoralPickupGround;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.DriveCloseToReef;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.OrbitReefToTag;
import frc.robot.subsystems.Navigation;

public class AutonTwoCoralTrough extends SequentialCommandGroup {
  public AutonTwoCoralTrough(RobotContainer bot, int tagid) {
            addCommands(
                        new DriveCloseToReef(bot.m_robotDrive, bot.m_nav),
                        Commands.parallel(new OrbitReefToTag(bot.m_robotDrive, bot.m_nav, tagid),
                                          bot.m_elevator.level1Command()
                                        ),
                        Commands.parallel(new GoToCommand(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(tagid, 0.05)),
                                          bot.m_gripper.extendCommand()
                                        ),
                        bot.m_gripper.openCommand(),
                        bot.m_gripper.retractCommand(),
                        bot.m_elevator.loadPosCommand(),

                        GoToCommand.relative(bot.m_robotDrive, bot.m_nav, -1.0, 0, 0),
                        new GoNearestIceCream(bot.m_robotDrive, bot.m_nav),
                        
                        new AutoCoralPickupGround(bot.m_robotDrive, bot.m_nav, bot.m_intake, bot.m_intakeArm, 1.3),
                        new AutoIntake(null, null, null, null),
                        new DriveCloseToReef(bot.m_robotDrive, bot.m_nav),
                        Commands.parallel(new OrbitReefToTag(bot.m_robotDrive, bot.m_nav, tagid),
                                          bot.m_elevator.level1Command()
                                        ),
                        Commands.parallel(new GoToCommand(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(tagid, 0.05)),
                                          bot.m_gripper.extendCommand()
                                        ),
                        bot.m_gripper.openCommand(),
                        bot.m_gripper.retractCommand(),
                        bot.m_elevator.loadPosCommand()
                      );

  }
}
