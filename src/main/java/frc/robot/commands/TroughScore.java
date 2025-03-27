// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

//TODO this whole thing need to be reconsidered

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TroughScore extends Command {
  /** Creates a new TroughScore. */
  IntakeArm m_arm;
  Intake m_intake;
  Timer m_timer;

  public TroughScore(IntakeArm arm, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_intake = intake;
    m_timer = new Timer();
    addRequirements(m_arm, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.troughPosition();
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_arm.atGoal()){
      m_intake.up();
      m_timer.restart();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_arm.elevatorPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(Constants.IntakeConstants.ejectTime);
  }
}
