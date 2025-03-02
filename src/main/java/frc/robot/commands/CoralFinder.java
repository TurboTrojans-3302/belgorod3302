// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralFinder extends ParallelRaceGroup {
  /** Creates a new CoralFinder. */
  
  public CoralFinder(RobotContainer bot, boolean limitLeft, boolean limitRight, double degreesLimit) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (true){
      addCommands(new SimpleCoralPicker(bot.m_nav, degreesLimit, limitLeft, limitRight),
    //back up if nothing is found to increase field of view, it should be found in much quicker than 2 seconds
    new RepeatCommand(new WaitCommand(2).andThen(GoToCommand.relative(bot.m_robotDrive, bot.m_nav, -1, 0, 0))));
  }
}
}