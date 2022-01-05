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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionStrafeCommand extends CommandBase {

    double startTime;
    // Returns true when the command should end.
    double finishedTx = 0;
    boolean isTargetVisible = false;

    double threshold = 3;
    double tx = 0;

    /**
     * Creates a new VisionStrafeCommand.
     */
    public VisionStrafeCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        Swerve.getInstance().snapToAngle(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (LimelightProcessor.getInstance().isTargetVisible()) {
            tx = LimelightProcessor.getInstance().getTX();

            if (Math.abs(tx) > threshold) {
                Swerve.getInstance().drive(Math.signum(tx), 0, 0, true);
            }
        }
        // LimelightProcessor.getInstance().refreshStandard(LimelightProcessor.Pipeline.DRIVER);
        // if ((!LimelightProcessor.getInstance().onTarget()) &&
        // (Timer.getFPGATimestamp() > startTime + 0.25)) {
        // LimelightProcessor.getInstance().powerPortOuterStrafe();
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().killVisionTranslationInput();
        SmartDashboard.putBoolean("Was target visible", isTargetVisible);
        SmartDashboard.putNumber("Last TX", finishedTx);
    }

    @Override
    public boolean isFinished() {
        // finishedTx = LimelightProcessor.getInstance().getTX();
        // isTargetVisible = LimelightProcessor.getInstance().isTargetVisible();
        return ((Timer.getFPGATimestamp() - startTime >= 5) || Math.abs(tx) < threshold);
    }
}
