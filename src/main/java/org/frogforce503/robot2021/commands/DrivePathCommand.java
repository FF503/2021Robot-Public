package org.frogforce503.robot2021.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frogforce503.lib.control.Trajectory;
import org.frogforce503.robot2021.RobotState;
import org.frogforce503.robot2021.RobotState.GameState;
import org.frogforce503.robot2021.paths.PathContainer;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

import java.util.function.Supplier;

public class DrivePathCommand extends CommandBase {
    private final Supplier<Trajectory> trajectorySupplier;
    String name;
    int i = 0;
    private double snapDelay;
    private double targetTheta;
    private double startTime = 0.0;
    private boolean useVision;

    public DrivePathCommand(Trajectory trajectory) {
        this(() -> trajectory);
        this.targetTheta = trajectory.getStopHeading();
        this.snapDelay = trajectory.getSnapDelay();
        this.name = trajectory.getName();
        this.useVision = trajectory.isVisionHeading();
    }

    public DrivePathCommand(PathContainer path) {
        this(path.buildTrajectory());
    }

    public DrivePathCommand(Supplier<Trajectory> trajectorySupplier) {
        this.trajectorySupplier = trajectorySupplier;
    }

    @Override
    public void initialize() {

        Trajectory trajectory = trajectorySupplier.get();
        if (!useVision) {
            Swerve.getInstance().updatePathAngularSetpoint(targetTheta, snapDelay, new Translation2d(0.0, 0.0), 2);
        }
        Swerve.getInstance().setAutonomousTrajectory(trajectory);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (useVision && Timer.getFPGATimestamp() - startTime > snapDelay) {
            if (LimelightProcessor.getInstance().isTargetVisible()) {
                LimelightProcessor.getInstance().powerPortOuterHeadingLock();
            } else {
                Swerve.getInstance().killVisionAngularInput();
                Swerve.getInstance().snapToAngle(0.0);
            }
        }
        SmartDashboard.putNumber("path update", i++);
        Swerve.getInstance().updateAutonomousTrajectoryFollowerOutput(RobotState.getInstance().getCurrentPose());
    }

    @Override
    public boolean isFinished() {
        // Only finish when the trajectory is completed
        boolean visionDone = !useVision || (Math.abs(LimelightProcessor.getInstance().getTX()) < 3);
        double timeout = 2000.0;
        return (RobotState.getInstance().getGameState() != GameState.AUTON
                || Timer.getFPGATimestamp() - startTime > timeout)
                || (visionDone && Swerve.getInstance().isTrajectoryFinished()
                && Swerve.getInstance().isAutonHeadingOnTarget());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        SmartDashboard.putNumber("Num Paths Ended", SmartDashboard.getNumber("Num Paths Ended", 0) + 1);
        Swerve.getInstance().killAutonTranslationInput();
    }

}