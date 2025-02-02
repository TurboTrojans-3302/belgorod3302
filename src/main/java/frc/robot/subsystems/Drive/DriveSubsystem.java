package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface DriveSubsystem {

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    Pose2d getPose();

    void drive(Translation2d translation, double rotation);

    void driveHeadingRobot(Translation2d translation, double rotation);

    void driveHeadingField(Translation2d translation, double rotation);

    void drive(Translation2d translation);

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit);

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    double getHeading();

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    double getTurnRate();

    double getSpeed();

    Double getDistanceToObjectMeters();

    boolean distanceMeasurmentGood();

    public void resetOdometry(Pose2d pose);
}