// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.frc2025.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class StartPositions {
    public static SendableChooser<Pose2d> getBlue(){
    
        SendableChooser<Pose2d> bluePoses = new SendableChooser<Pose2d>();
        bluePoses.setDefaultOption("Zero-Zero", Pose2d.kZero);
        bluePoses.addOption("Left Blue Cage",  new Pose2d(FieldConstants.startingLineX, FieldConstants.Barge.farCage.getY(), Rotation2d.k180deg));
        bluePoses.addOption("Mid Blue Cage",   new Pose2d(FieldConstants.startingLineX, FieldConstants.Barge.middleCage.getY(), Rotation2d.k180deg));
        bluePoses.addOption("Right Blue Cage", new Pose2d(FieldConstants.startingLineX, FieldConstants.Barge.closeCage.getY(), Rotation2d.k180deg));
        bluePoses.addOption("Center Field",    new Pose2d(FieldConstants.startingLineX, FieldConstants.fieldWidth / 2, Rotation2d.k180deg));
        bluePoses.addOption("Left Red Cage",   new Pose2d(FieldConstants.startingLineX, 2.976, Rotation2d.k180deg));
        bluePoses.addOption("Mid Red Cage",    new Pose2d(FieldConstants.startingLineX, 1.886, Rotation2d.k180deg));
        bluePoses.addOption("Right Red Cage",  new Pose2d(FieldConstants.startingLineX, 0.786, Rotation2d.k180deg));
        
        return bluePoses;
    }

    public static SendableChooser<Pose2d> getRed(){
        double redStartLine = FieldConstants.fieldLength - FieldConstants.startingLineX;

        SendableChooser<Pose2d> redPoses = new SendableChooser<Pose2d>();
        redPoses.setDefaultOption("Zero-Zero", Pose2d.kZero);
        redPoses.addOption("Right Blue Cage", new Pose2d(redStartLine, FieldConstants.Barge.farCage.getY(), Rotation2d.kZero));
        redPoses.addOption("Mid Blue Cage",   new Pose2d(redStartLine, FieldConstants.Barge.middleCage.getY(), Rotation2d.kZero));
        redPoses.addOption("Left Blue Cage",  new Pose2d(redStartLine, FieldConstants.Barge.closeCage.getY(), Rotation2d.kZero));
        redPoses.addOption("Center Field",    new Pose2d(redStartLine, FieldConstants.fieldWidth / 2, Rotation2d.kZero));
        redPoses.addOption("Right Red Cage",  new Pose2d(redStartLine, 2.976, Rotation2d.kZero));
        redPoses.addOption("Mid Red Cage",    new Pose2d(redStartLine, 1.886, Rotation2d.kZero));
        redPoses.addOption("Left Red Cage",   new Pose2d(redStartLine, 0.786, Rotation2d.kZero));
        
        return redPoses;
    }
}
