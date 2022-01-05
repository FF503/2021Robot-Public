/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021.commands;

import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionAimCommand extends CommandBase {

    double startTime;

    /**
     * Creates a new visionAimCommand.
     */
    public VisionAimCommand() {
        // this.useDistanceControl = useDistanceControl;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println("vision running aim");
        LimelightProcessor.getInstance().powerPortOuterHeadingLock();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().killVisionAngularInput();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - startTime > 1.0) && (!LimelightProcessor.getInstance().isTargetVisible()
                || (LimelightProcessor.getInstance().onTarget())));
    }
}