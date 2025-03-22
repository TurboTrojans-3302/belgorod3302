// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Navigation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCoralPickupGround extends ParallelCommandGroup {
  /** Creates a new AutoCoralPickupGround. */

  public AutoCoralPickupGround(DriveSubsystem drive, Navigation nav, Intake intake, IntakeArm arm, double distanceForward) {
  
    addCommands(new InstantCommand(()->{
                        arm.floorPosition();
                        intake.in();
                    }),
                GoToCommand.relative(drive, nav, distanceForward, 0, 0));
  }
}
