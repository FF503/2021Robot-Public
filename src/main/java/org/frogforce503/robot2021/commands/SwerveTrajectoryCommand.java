// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2021.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;

import java.io.IOException;
import java.nio.file.Path;

public class SwerveTrajectoryCommand extends CommandBase {

    private Trajectory mTrajectory = null;
    private boolean mResetPose = true;

    /**
     * Creates a new SwerveTrajectoryFollowingCommand.
     */
    public SwerveTrajectoryCommand(Trajectory trajectory) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.mTrajectory = trajectory;
    }

    public SwerveTrajectoryCommand(String jsonName) {

        String trajectoryJson = "AutonJSONs/AutoNavPaths/output/" + jsonName + ".wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);

            this.mTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJson, ex.getStackTrace());
        }
    }

    public SwerveTrajectoryCommand(String jsonName, boolean resetPose) {

        this(jsonName);
        this.mResetPose = resetPose;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Swerve.getInstance().setHolonomicFollowerTrajectory(mTrajectory, mResetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().setRequestedDriverTranslation(new Translation2d());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Swerve.getInstance().isHolonomicFollowerFinished();
    }

}
