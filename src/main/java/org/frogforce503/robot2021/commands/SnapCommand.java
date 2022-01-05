/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2021.RobotState;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;

public class SnapCommand extends CommandBase {
    double angle;
    double tolerance;
    boolean manualTolerance;

    /**
     * Creates a new visionAimCommand.
     */
    public SnapCommand(double angle) {
        this.angle = angle;
        this.manualTolerance = false;
    }

    public SnapCommand(double angle, double tolerance) {
        this.angle = angle;
        this.tolerance = tolerance;
        this.manualTolerance = true;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Swerve.getInstance().updatePathAngularSetpoint(angle, 0.0, new Translation2d(0.0, 0.0), 2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().updatePathAngularSetpoint(RobotState.getInstance().getCurrentTheta(), 0.0,
                RobotState.getInstance().getCurrentPose().getTranslation(), 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (this.manualTolerance) {
            System.out.println("Manual Tolerance... " + RobotState.getInstance().getCurrentTheta());
            return (Math.abs((Util.boundAngleNeg180to180Degrees(
                    RobotState.getInstance().getCurrentTheta() - angle))) < this.tolerance);
        }
        return Swerve.getInstance().isAutonHeadingOnTarget();
    }
}