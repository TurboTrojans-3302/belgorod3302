/*
 * Deploy the intake and move forward slowly until a coral is picked up
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Navigation;

public class AutoCoralPickupGround extends SequentialCommandGroup {
  /** Creates a new AutoCoralPickupGround. */

  public AutoCoralPickupGround(DriveSubsystem drive, Navigation nav, Intake intake, IntakeArm arm, double distanceForward) {
  
    addCommands(arm.floorPositionCommand(),
                intake.startPickupCommand(),
                GoToCommand.relative(drive, nav, distanceForward, 0, 0, 0.3)
                    .until(intake::lowerObjectDetected)
                );
                
  }
}
