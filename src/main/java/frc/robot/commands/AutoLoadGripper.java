// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoLoadGripper extends Command {
  private Intake intake;
  private Gripper gripper;

  /** Creates a new AutoLoadGripper. */
  public AutoLoadGripper(Intake intake, Gripper gripper) {
    this.intake = intake;
    this.gripper = gripper;
    addRequirements(intake, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripper.openGripper();
    intake.loadGripper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gripper.objectInGripper()){
      gripper.closeGripper();
      intake.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripper.objectInGripper() && gripper.isGripperClosed();
  }
}
