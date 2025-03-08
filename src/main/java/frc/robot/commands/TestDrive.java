// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestDrive extends Command {
  private DriveSubsystem m_robotDrive;
  private XboxController m_driverController;

  /** Creates a new TestDrive. */
  public TestDrive(DriveSubsystem robotDrive, XboxController driverController) {
      m_driverController = driverController;
      m_robotDrive = robotDrive;
      addRequirements(m_robotDrive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double drive = -m_driverController.getLeftY();
    double steer = -m_driverController.getRightY();
    

    m_robotDrive.testSetAll(drive, steer);

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
