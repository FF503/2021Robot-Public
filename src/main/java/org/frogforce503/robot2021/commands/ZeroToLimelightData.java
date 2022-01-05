/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frogforce503.robot2021.loops.FroggyPoseController;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

public class ZeroToLimelightData extends CommandBase {

    /**
     * Creates a auton ready to shoot command
     */
    public ZeroToLimelightData() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Translation2d pose = new Translation2d(150.0 - LimelightProcessor.getInstance().distFromTarget(), 0.0);
        FroggyPoseController.resetPose(new Pose2d(pose, new Rotation2d()));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}