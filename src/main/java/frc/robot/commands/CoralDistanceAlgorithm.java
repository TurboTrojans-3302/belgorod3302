// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralDistanceAlgorithm extends Command {
  /** Creates a new CoralDistanceAlgorithm. */
  Navigation m_nav;
  boolean coralRight;
  boolean coralLeft; 
  String cameraName = Constants.LimelightConstants.name;
  double targetTX;
  double distanceToTarget;
  NetworkTable table = NetworkTableInstance.getDefault().getTable(cameraName);
  double[] txValues;
  int[] validTargets;
  double timeToTarget = 999;
  int targetID;
  double angleTolerance;
  boolean targetChosen = false;

  double speedLimit;
  double accelerationLimit;
  double angularSpeedLimit;
  
  public CoralDistanceAlgorithm(Navigation nav, boolean rightOnly, boolean leftOnly, double possibleAngleTolerance, double maxSpeed, double maxAcceleration, double maxAngularSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_nav = nav;
    coralRight = rightOnly;
    coralLeft = leftOnly;
    angleTolerance = possibleAngleTolerance;
    speedLimit = maxSpeed;
    accelerationLimit = maxAcceleration;
    angularSpeedLimit = maxAngularSpeed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.getRawFiducials(cameraName);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 

    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(cameraName);
    txValues = table.getEntry("tx").getDoubleArray(new double[0]);

    if (LimelightHelpers.getTargetCount(cameraName) > 1){
      for (int i = 0; i > LimelightHelpers.getTargetCount(cameraName); i++){
        
        double tx = txValues[i];



        if ((coralRight && (tx < (0 - angleTolerance))) || (coralLeft && (tx > 0 + angleTolerance))){
          validTargets[i] = 0; //not valid
        } else {
          validTargets[i] = 1; //valid

        } //if there is no limit to a certain direction of rotation, then all targets are deemed valid


      }

      for (int i = 0; i > validTargets.length; i++){
        //validtargets length will still be the same as the limelight helpers target count unless new targets have been added to the frame of reference.
        if (validTargets[i] == 1){
          //calculating time to target
          double distance = fiducials[i].distToCamera;
          double tx = txValues[i];
         
          //approximate time in seconds for distance to target is calculated below
          
          //TODO is this not working to set an initial and goal position because it says these are not used
          TrapezoidProfile.State initialPosition = new State(0.0, 0.0);
          TrapezoidProfile.State goalPosition = new State(distance, 0.0);
          TrapezoidProfile.Constraints constraints = new Constraints(speedLimit, accelerationLimit);
          TrapezoidProfile m_trapezoidPrediction = new TrapezoidProfile(constraints);
          

          //calculate rotation time
          double angularSpeed = angularSpeedLimit * (180/Math.PI); //convert to degrees

          double rotationTime = Math.abs(tx/angularSpeed);
          double calculatedTargetTime = m_trapezoidPrediction.totalTime() + rotationTime; //seconds

          if (calculatedTargetTime < timeToTarget){
            timeToTarget = m_trapezoidPrediction.totalTime();
            targetID = i;
            targetTX = tx;
            distanceToTarget = distance;

          }
        }

      }

      m_nav.setTargetTX(targetTX);
      m_nav.setTargetDistanceFiducial(distanceToTarget);
      System.out.println("Time to target: " + timeToTarget);
      targetChosen = true;


    } else if (LimelightHelpers.getTargetCount(cameraName) == 1){
      targetTX = LimelightHelpers.getTX(cameraName);
      m_nav.setTargetTX(targetTX);
      m_nav.setTargetDistanceFiducial(fiducials[0].distToCamera);
      System.out.println("Only one target");
      targetChosen = true;

    } else {
      System.out.println("No valid targets");

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetChosen;
  }
}
