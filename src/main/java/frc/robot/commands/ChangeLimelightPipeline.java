// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Navigation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChangeLimelightPipeline extends InstantCommand {
  Navigation m_nav;
  int pipeline;
  public ChangeLimelightPipeline(Navigation nav, int pipelineID) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_nav = nav;
    pipeline = pipelineID;
    addRequirements(m_nav);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_nav.changePipeline(pipeline);
  }
}
