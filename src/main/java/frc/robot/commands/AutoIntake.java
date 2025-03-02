// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends Command {
  /** Creates a new AutoIntake. */
  Intake m_intake;
  IntakeArm m_arm;
  private Gripper m_gripper;
  private Elevator m_Elevator;

  public AutoIntake(Intake intake, IntakeArm arm, Gripper gripper, Elevator elevator) {
    m_arm = arm;
    m_intake = intake;
    this.m_gripper = gripper;
    this.m_Elevator = elevator;
    addRequirements(m_intake, m_arm, m_Elevator, m_gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.openGripper();
    m_gripper.retractGripper();
    m_Elevator.setPosition(Constants.ElevatorConstants.kLoadPosition);
    m_intake.stop();
    m_arm.elevatorPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setLowerSpeed(0);
    m_intake.setUpperSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atSetpoint() && m_Elevator.atSetpoint();
  }
}
