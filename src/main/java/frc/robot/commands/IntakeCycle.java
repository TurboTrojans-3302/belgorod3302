// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCycle extends SequentialCommandGroup {
  /** Creates a new IntakeCycle. */

  public IntakeCycle(Intake intake, IntakeArm arm, Gripper gripper, Elevator elevator) {
   
    addCommands(new AutoIntake(intake, arm, gripper, elevator), 
                new AutoLoadGripper(intake, gripper)
                );
  }
}
