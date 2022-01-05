package org.frogforce503.robot2021.loops;

import org.frogforce503.robot2021.Robot;
import org.frogforce503.robot2021.RobotState;
import org.frogforce503.robot2021.subsystems.swerve.Pigeon;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Pose controller that calculates translation coordinates based on odometry and
 * uses yaw from the Pigeon IMU
 */
public class FroggyPoseController {
    private static final SwerveDriveKinematics kinematics;
    private static final SwerveDriveOdometry mOdometry;
    private static final Swerve mSwerve;

    static {
        kinematics = new SwerveDriveKinematics(Robot.bot.kModulePositions);
        mOdometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0));
        mSwerve = Swerve.getInstance();
    }

    public static synchronized void updateOdometry() {
        final double robotAngle = Pigeon.getInstance().getYaw();
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = mSwerve.getModules().get(i).getState();
        }

        var chassisSpeeds = kinematics.toChassisSpeeds(moduleStates);

        // robot calculated X, Y velocity
        double dX = chassisSpeeds.vxMetersPerSecond;
        double dY = chassisSpeeds.vyMetersPerSecond;

        SmartDashboard.putNumber("Robot dX", dX);
        SmartDashboard.putNumber("Robot dY", dY);
        SmartDashboard.putNumber("Robot dTheta", Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));

        double velMag = Math.sqrt((dX * dX) + (dY * dY)); // magnitude of the velocity vector

        double velAngle = Math.atan2(dY, dX) + Math.toRadians(-robotAngle % 360); // angle of velocity
        // vector +
        // actual robot angle
        SmartDashboard.putNumber("Field Centric Robot velAngle", Math.toDegrees(velAngle));
        // field centric X and Y acceleration
        double fieldCentricDX = velMag * Math.cos(velAngle);
        double fieldCentricDY = velMag * Math.sin(velAngle);

        SmartDashboard.putNumber("Field Centric Robot dX", fieldCentricDX);
        SmartDashboard.putNumber("Field Centric Robot dY", fieldCentricDY);

        RobotState.getInstance().setCurrentSpeeds(chassisSpeeds);
        RobotState.getInstance().setFieldCentricSpeeds(fieldCentricDX, fieldCentricDY);
        RobotState.getInstance()
                .setCurrentPose(mOdometry.update(Rotation2d.fromDegrees(robotAngle).unaryMinus(), moduleStates));
    }

    public static synchronized void resetPose(final Pose2d pose) {
        while (!mOdometry.getPoseMeters().equals(pose)) {
            mOdometry.resetPosition(pose, Rotation2d.fromDegrees(0));
        }
    }

    public static synchronized void resetPose() {
        while (mOdometry.getPoseMeters().getTranslation().getNorm() != 0.0) {
            mOdometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
        }

    }

    public static synchronized void outputPoseToDashboard() {
        final Pose2d robotPose = RobotState.getInstance().getCurrentPose();

        SmartDashboard.putNumber("Robot X: ", robotPose.getTranslation().getX());
        SmartDashboard.putNumber("Robot Y: ", robotPose.getTranslation().getY());

    }

}