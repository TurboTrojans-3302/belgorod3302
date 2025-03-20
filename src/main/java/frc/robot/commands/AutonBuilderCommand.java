// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutonBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonBuilderCommand extends SequentialCommandGroup {
  /** Creates a new AutonBuilderCommand. */
  AutonBuilder m_builder;
  public AutonBuilderCommand(AutonBuilder builder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_builder = builder;
    for (int i = 0; i == m_builder.getActionCount(); i++){
    addCommands(m_builder.getActionStep(i));
  }
  }
}
