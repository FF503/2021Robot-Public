package org.frogforce503.robot2021.commands;

import org.frogforce503.robot2021.RobotState;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleVisionSnap extends CommandBase {

    double startTime;

    /**
     * Creates a new visionAimCommand.
     */
    public ToggleVisionSnap() {
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
        double angle = LimelightProcessor.getInstance().getPowerPortTurretError()
                + RobotState.getInstance().getCurrentTheta();
        Swerve.getInstance().snapToAngle(angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Swerve.getInstance().killAnchorMode();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - startTime > 1.0) && (!LimelightProcessor.getInstance().isTargetVisible()
                || (LimelightProcessor.getInstance().onTarget())));
    }
}