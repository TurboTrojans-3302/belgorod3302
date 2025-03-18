// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral extends Command {
  /** Creates a new Coral. */
  IntakeArm m_arm;
  Intake m_intake;
  public Coral(IntakeArm arm, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_arm = arm;
    addRequirements(m_intake, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stop();
    m_arm.elevatorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atSetpoint();
  }
}
