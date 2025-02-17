// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Navigation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCoralPickupGround extends ParallelCommandGroup {
  /** Creates a new AutoCoralPickupGround. */
  DriveSubsystem m_drive;
  Navigation m_nav;
  Intake m_intake;
  double forwardMovement;
  public AutoCoralPickupGround(DriveSubsystem drive, Navigation nav, Intake intake, double distanceForward) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drive = drive;
    m_nav = nav;
    m_intake = intake;
    forwardMovement = distanceForward;
    addCommands(GoToCommand.relative(drive, nav, forwardMovement, 0, 0), new AutoIntake(m_intake));
  }
}
