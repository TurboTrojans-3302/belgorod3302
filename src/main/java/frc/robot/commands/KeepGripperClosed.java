// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KeepGripperClosed extends Command {
  /** Creates a new KeepGripperClosed. */
  Gripper m_gripper;
  public KeepGripperClosed(Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gripper = gripper;
    addRequirements(m_gripper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.closeGripper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // the command should never end until a command it is grouped with finishes
  @Override
  public boolean isFinished() {

    return false;
  }
}
