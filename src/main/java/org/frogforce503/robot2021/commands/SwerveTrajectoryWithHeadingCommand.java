// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2021.commands;

import java.io.IOException;

import org.frogforce503.lib.motion.HolonomicTrajectory;
import org.frogforce503.lib.motion.HolonomicTrajectoryUtil;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveTrajectoryWithHeadingCommand extends CommandBase {

    private HolonomicTrajectory mTrajectory = null;
    private boolean mResetPose = true;

    /**
     * Creates a new SwerveTrajectoryFollowingCommand.
     */
    public SwerveTrajectoryWithHeadingCommand(HolonomicTrajectory trajectory) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.mTrajectory = trajectory;
    }

    public SwerveTrajectoryWithHeadingCommand(String csvName) {

        try {
            this.mTrajectory = HolonomicTrajectoryUtil.fromPathPlannerCsv(csvName);

        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + csvName, ex.getStackTrace());
        }
    }

    public SwerveTrajectoryWithHeadingCommand(String csvName, boolean resetPose) {
        this(csvName);
        this.mResetPose = resetPose;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Swerve.getInstance().setHolonomicHeadingTrajectory(mTrajectory, mResetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().killAutonTranslationInput();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Swerve.getInstance().isHolonomicFollowerFinished();
    }

}
