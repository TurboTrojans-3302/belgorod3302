// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAndGripper extends Command {
  /** Creates a new IntakeAndGripper. */
  Intake m_intake;
  Gripper m_gripper;

  public IntakeAndGripper(Intake intake, Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_gripper = gripper;
    addRequirements(m_intake, m_gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.openGripper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_gripper.isGripperOpen()){ 
      m_intake.setSpeed(-Constants.IntakeConstants.intakeSpeedAuto); //if the gripper is open the coral can be transferred from the intake because this command is called after the arm is at the correct angle.
    }

    //object needs to leave intake and enter gripper
    if (!m_intake.objectDetected && m_gripper.objectInGripper()){
      new WaitCommand(0.4); //wait for coral transfer
      m_gripper.closeGripper();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gripper.isGripperClosed();
  }
}
