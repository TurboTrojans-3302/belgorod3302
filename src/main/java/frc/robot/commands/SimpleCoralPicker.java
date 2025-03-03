// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleCoralPicker extends Command {
  /** Creates a new SimpleCoralPicker. */
  private final String cameraName = "limelight";
  Navigation m_nav;
  LimelightResults results;
  double targetTX;
  double targetArea = 0.0;
  double degreeLimit;
  int targetID;
  double confidenceThreshold = 0.75;
  boolean finished;
  boolean leftLimit;
  boolean rightLimit;

  public SimpleCoralPicker(Navigation nav, double degreesLimit, boolean applyLeft, boolean applyRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_nav = nav;
    addRequirements(m_nav);
    leftLimit = applyLeft;
    rightLimit = applyRight;
   
    //left is negative tx
    if (leftLimit){
      degreeLimit = -degreesLimit;
    }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    results = LimelightHelpers.getLatestResults(cameraName);
    if (results.targets_Detector.length == 1){
      LimelightTarget_Detector target = results.targets_Detector[0];
      targetTX = target.tx;
       m_nav.setTargetTX(targetTX);
       finished = true;
  }

 


    if (results.targets_Detector.length > 1){

      for (int i = 0; i < results.targets_Detector.length; i++){

        double targetAreaInstance = results.targets_Detector[i].ta;
        double targetTXInstance = results.targets_Detector[i].tx;
        if (leftLimit && targetTXInstance < degreeLimit){
          targetAreaInstance = 0;
          //target not considered
        } else if (rightLimit && targetTXInstance > degreeLimit) {
          targetAreaInstance = 0;
        }

        if (targetAreaInstance > targetArea){
          targetArea = targetAreaInstance;
          targetID = i;
        }
      }

      targetTX = results.targets_Detector[targetID].tx;
      m_nav.setTargetTX(targetTX);
      finished = true;

    } else {
      System.out.println("Searching for target");
      //back up/drive forward or rotate robot
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
