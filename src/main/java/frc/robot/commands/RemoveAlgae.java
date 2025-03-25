// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgae extends Command {
  /** Creates a new RemoveAlgae. */
  RobotContainer bot;
  boolean readyToRemove = false;
  boolean finished = false;
  double timeSeconds = 0.0;
  public RemoveAlgae(RobotContainer robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    bot = robot; 
    addRequirements(bot.m_elevator, bot.m_gripper, bot.m_nav, bot.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      bot.m_elevator.setPosition(Constants.ElevatorConstants.Algae);
      bot.m_gripper.extendGripper();
      
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (bot.m_elevator.atSetpoint() && bot.m_gripper.isGripperFullyExtended()){
      bot.m_elevator.setPosition(Constants.ElevatorConstants.kLevel4); //TODO does the elevator have to be raised this high?
      //TODO does there need to be time in between starting the elevator and moving backwards

    for (int i = 0; i > 100; i++){
      timeSeconds = timeSeconds + 0.020;
    if (timeSeconds == 1.0){
      GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 1.0, 0.0, 0.0);
    }

   }
  }
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bot.m_elevator.stop();
    bot.m_gripper.retractGripper();
    //probably shouldnt move the eevator down automatically just in case.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (finished && bot.m_elevator.atSetpoint() && (timeSeconds > 2.0));
  }
}
