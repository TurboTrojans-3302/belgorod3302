// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadGripper extends Command {
  private RobotContainer robot;
  
    /** Creates a new LoadGripper. */
    
    public LoadGripper(RobotContainer robot) {
      this.robot = robot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robot.m_intake, robot.m_intakeArm, robot.m_gripper, robot.m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robot.m_gripper.openGripper();
    robot.m_elevator.setPosition(Constants.ElevatorConstants.kLoadPosition);
    robot.m_intakeArm.elevatorPosition();
    robot.m_intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (robot.m_elevator.atSetpoint() && robot.m_gripper.isGripperOpen() && robot.m_intakeArm.atSetpoint()){
      robot.m_intake.loadGripper();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robot.m_intake.stop();
    robot.m_gripper.closeGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robot.m_gripper.objectInGripper();
  }
}
