// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDrive extends Command {
  private DriveSubsystem m_robotDrive;
  private XboxController m_driverController;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveSubsystem robotDrive, XboxController driverController) {
    m_driverController = driverController;
    m_robotDrive = robotDrive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.setAll(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedScale;
    if (m_driverController.getRightTriggerAxis() < 0.5) {
      speedScale = 1;
    } else {
      speedScale = 0.5;
    }

    if(m_driverController.getRightBumperButton()) {
      m_robotDrive.driveRobotOriented(
        stick2speed(speedScale * -1.0 * m_driverController.getLeftX()),
        stick2speed(speedScale * -1.0 * m_driverController.getLeftY()),
        stick2speed(speedScale * -1.0 * m_driverController.getRightX()));
    } else {
      m_robotDrive.driveFieldOriented(
        stick2speed(speedScale * -1.0 * m_driverController.getLeftX()),
        stick2speed(speedScale * -1.0 * m_driverController.getLeftY()),
        stick2speed(speedScale * -1.0 * m_driverController.getRightX()));
    }
    
  }

  // applies deadband and scaling to raw stick value
  private double stick2speed(double stickValue) {
    return Math.signum(stickValue) * Math.pow(MathUtil.applyDeadband(stickValue, OIConstants.kDriveDeadband), 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
