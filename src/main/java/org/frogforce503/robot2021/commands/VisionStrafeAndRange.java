/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

public class VisionStrafeAndRange extends CommandBase {
    double startTime;
    double distance;
    // Returns true when the command should end.
    double finishedTx = 0;
    boolean isTargetVisible = false;

    // Called every time the scheduler runs while the command is scheduled.

    /**
     * Creates a new VisionStrafeCommand.
     */
    public VisionStrafeAndRange(double distance) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.distance = distance;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

        LimelightProcessor.getInstance().refreshStandard(LimelightProcessor.Pipeline.POWER_PORT);
        LimelightProcessor.getInstance().powerPortRangeAndStrafe(distance);

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
        finishedTx = LimelightProcessor.getInstance().getTX();
        isTargetVisible = LimelightProcessor.getInstance().isTargetVisible();

        return ((Timer.getFPGATimestamp() - startTime > 1.0)
                && (!isTargetVisible || (LimelightProcessor.getInstance().onTarget()
                && LimelightProcessor.getInstance().getDistanceOnTarget())));
    }
}
