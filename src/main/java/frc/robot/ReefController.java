// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Navigation.ReefPole;

/** Add your docs here. */
public class ReefController {

    private static final int[] BlueTagId = {-1, 21, 20, 20, 19, 19, 18, 18, 17, 17, 22, 22, 21};
    private static final int[] RedTagId =  {-1, 10, 11, 11,  6,  6,  7,  7,  8,  8,  9,  9, 10};
    private static final double distance = 1.0;

    private static final Pose2d[] BlueReefPosition = {
        Pose2d.kZero,
        Navigation.getPose2dInFrontOfTag(BlueTagId[1],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(BlueTagId[2],  distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(BlueTagId[3],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(BlueTagId[4],  distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(BlueTagId[5],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(BlueTagId[6],  distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(BlueTagId[7],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(BlueTagId[8],  distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(BlueTagId[9],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(BlueTagId[10], distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(BlueTagId[11], distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(BlueTagId[12], distance, ReefPole.right)
    };

    private static final Pose2d[] RedReefPosition = {
        Pose2d.kZero,
        Navigation.getPose2dInFrontOfTag(RedTagId[1],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(RedTagId[2],  distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(RedTagId[3],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(RedTagId[4],  distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(RedTagId[5],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(RedTagId[6],  distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(RedTagId[7],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(RedTagId[8],  distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(RedTagId[9],  distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(RedTagId[10], distance, ReefPole.right),
        Navigation.getPose2dInFrontOfTag(RedTagId[11], distance, ReefPole.left),
        Navigation.getPose2dInFrontOfTag(RedTagId[12], distance, ReefPole.right)
    };

    private GenericHID m_controller;
    private int value = 1;

    public ReefController(int controllerPort ){
        m_controller = new GenericHID(controllerPort);

    }

    public int getSwitchPosition(){
        for(int i = 1; i < 13; i++){
            if(m_controller.getRawButton(i)){
                value = i;
            }
        }
        return value;
    }

    public int getAprilTagId(){
        if(Robot.alliance == Alliance.Red){
            return RedTagId[getSwitchPosition()];
        } else {
            return BlueTagId[getSwitchPosition()];
        }
    }

    public Pose2d getTargetPose2d(){
        if(Robot.alliance == Alliance.Red){
            return RedReefPosition[getSwitchPosition()];
        } else {
            return BlueReefPosition[getSwitchPosition()];
        }
    }
}
