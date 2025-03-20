// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Navigation.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithPole extends Command {
  /** Creates a new AlignWithPole. */
  RobotContainer bot;
  Navigation m_nav;
  Boolean left = false;
  Boolean right = false;
  String cameraName = "limelight";
  public AlignWithPole(Boolean moveLeft) {
    // Use addRequirements() here to declare subsystem dependencies.
  bot = RobotContainer.getInstance();
  m_nav = bot.m_nav;

  if (moveLeft){
    left = true;
  } else {
    right = true;
  }




  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (LimelightHelpers.getTV(cameraName)){
      if (left){
      Navigation.getPose2dInFrontOfTag((int) LimelightHelpers.getFiducialID(cameraName), 0.2, ReefPole.left);}
      else {
      Navigation.getPose2dInFrontOfTag((int) LimelightHelpers.getFiducialID(cameraName), 0.2, ReefPole.right);  
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
