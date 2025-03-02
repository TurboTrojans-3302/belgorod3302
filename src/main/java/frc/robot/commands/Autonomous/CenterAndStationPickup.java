// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoCoralPickupGround;
import frc.robot.commands.ChangeLimelightPipeline;
import frc.robot.commands.CoralChaser;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.IntakeCycle;
import frc.robot.commands.MoveRobotAndElevator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Navigation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterAndStationPickup extends SequentialCommandGroup {
  /** Creates a new WallSideGroundScoreBlue. */
  DriveSubsystem m_drive;
  Navigation m_nav;
  Elevator m_elevator;
  Gripper m_gripper;
  IntakeArm m_intakeArm;
  Intake m_intake;
  double position1;
  double position2;
  double poleOffset1;
  double poleOffset2;
  int aprilTagReef;
  int aprilTagStation;
  double stationRightXOffset;
  double stationRightYOffset;
  Pose2d aprilTagPoseReef;
  Pose2d aprilTagPoseStation;
  Alliance kAlliance;
  boolean limitPickupRight = false;
  boolean limitPickupLeft = false;
  double angleTolerance;
  boolean stationLeft;
  boolean stationRight;

  public CenterAndStationPickup(RobotContainer bot, double elevatorScoringPosition1, double elevatorScoringPosition2, boolean leftFirst, double angleToleranceOfStationPickup, String alliance, boolean StationLeft, boolean StationRight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drive = bot.m_robotDrive;
    m_nav = bot.m_nav;
    m_elevator = bot.m_elevator;
    m_gripper = bot.m_gripper;
    m_intakeArm = bot.m_intakeArm;
    m_intake = bot.m_intake;
    //customisable scoring positions so we dont have to make so many commands
    position1 = elevatorScoringPosition1;
    position2 = elevatorScoringPosition2;

    angleTolerance = angleToleranceOfStationPickup;
    stationLeft = StationLeft;
    stationRight = StationRight;

    if (alliance == "Blue" || alliance == "blue"){
      aprilTagReef = 20;
      aprilTagStation = 13;
      stationRightXOffset = Constants.FieldConstants.poseOffsetStationRightX;
      stationRightYOffset = Constants.FieldConstants.poseOffsetStationRightY;
    } else {
      //red
      aprilTagReef = 11;
      aprilTagStation = 1;
      stationRightXOffset = -Constants.FieldConstants.poseOffsetStationRightX;
      stationRightYOffset = -Constants.FieldConstants.poseOffsetStationRightY;
    }
    
    

    //configurable pole positions so we can create distance between us and our teammates if need be instead of going to the same pole first each time we run auto
    if (leftFirst){
      poleOffset1 = Constants.FieldConstants.yOffsetReefPoleLeft;
      poleOffset2 = Constants.FieldConstants.yOffsetReefPoleRight;
     
    } else {
      poleOffset1 = Constants.FieldConstants.yOffsetReefPoleRight;
      poleOffset2 = Constants.FieldConstants.yOffsetReefPoleLeft;
      
    }

    if (stationLeft){
      limitPickupRight = true;
    } else if (stationRight) {
      limitPickupLeft = true;
    }
    aprilTagPoseReef = Navigation.getPose2dInFrontOfTag(aprilTagReef, 1.0);
    aprilTagPoseStation = Navigation.getPose2dInFrontOfTag(aprilTagStation, 2.5);
    addCommands(new MoveRobotAndElevator(m_drive, m_nav, m_elevator, aprilTagPoseReef, elevatorScoringPosition1), 
                new DriveToAprilTag(m_drive, m_nav, aprilTagReef),
                GoToCommand.relative(m_drive, m_nav, 0.0, poleOffset1, 0.0), //I think this is the only time it makes sense to use relative so we dont have to have coordinates which are different for each side of the reef.
                m_gripper.extendCommand(),
                new WaitCommand(0.2),
                m_gripper.openCommand(),
                new WaitCommand(0.2),
                new MoveRobotAndElevator(m_drive, m_nav, m_elevator, new Pose2d(aprilTagPoseStation.getX() + Constants.FieldConstants.poseOffsetStationRightX, aprilTagPoseStation.getY() + Constants.FieldConstants.poseOffsetStationRightY, aprilTagPoseStation.getRotation()), Constants.ElevatorConstants.kPickupLevel),
                //the change to the pose is supposed to move the robot so it is facing the right side of the station, to minimize the chances of getting in someones way
                new AutoCoralPickupGround(m_drive, m_nav, m_intake, m_intakeArm, 1.0)
                        .until(()->m_intake.lowerObjectDetected()), //just drives forward with intake on, I want something that isn't just guesswork, maybe color detection
                new IntakeCycle(m_intake, m_intakeArm, m_gripper, m_elevator), //intake cycle does everything from spinning the intake, moving the arm, and transferring from the arm to the gripper, it skips the steps that have already been done
                new MoveRobotAndElevator(m_drive, m_nav, m_elevator, aprilTagPoseReef, elevatorScoringPosition2),
                new DriveToAprilTag(m_drive, m_nav, aprilTagReef),
                GoToCommand.relative(m_drive, m_nav, 0.0, poleOffset2, 0.0),
                m_gripper.extendCommand(),
                new WaitCommand(0.2),
                m_gripper.openCommand()
                );
}
}