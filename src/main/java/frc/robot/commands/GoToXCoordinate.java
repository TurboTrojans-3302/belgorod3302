/*
 *  Move in the X direction only, to a given X coordinate
 */
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToXCoordinate extends GoToCommand {
  private Double xDest;
  
    /** Creates a new GoToXCoordinate. */
    public GoToXCoordinate(DriveSubsystem drive, Navigation nav, Double XFieldCoordinate) {
      super(drive, nav);
      xDest = XFieldCoordinate;  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double x = m_nav.getPose().getX();
    double y = m_nav.getPose().getY();
    m_dest = new Pose2d(xDest, y, ( x > xDest ? Rotation2d.k180deg : Rotation2d.kZero ));
  }
}
