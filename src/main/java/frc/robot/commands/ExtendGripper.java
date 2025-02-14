// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ExtendGripper extends Command {
  /** Creates a new ExtendGripper. */

  Gripper m_gripper;
  

  public ExtendGripper(Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gripper = gripper;
    addRequirements(m_gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_gripper.extendGripper();
  }

  @Override
  public boolean isFinished(){
    return m_gripper.isGripperFullyExtended();
  }

  
}
