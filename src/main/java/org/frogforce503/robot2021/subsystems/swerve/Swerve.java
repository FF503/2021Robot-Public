package org.frogforce503.robot2021.subsystems.swerve;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.frogforce503.lib.control.HolonomicPurePursuitTrajectoryFollower;
import org.frogforce503.lib.control.PidConstants;
import org.frogforce503.lib.control.Trajectory;
import org.frogforce503.lib.math.RigidTransform2;
import org.frogforce503.lib.math.Rotation2;
import org.frogforce503.lib.math.Vector2;
import org.frogforce503.lib.motion.HolonomicTrajectory;
import org.frogforce503.lib.util.DrivetrainFeedforwardConstants;
import org.frogforce503.lib.util.FrogPIDF;
import org.frogforce503.lib.util.HolonomicDriveSignal;
import org.frogforce503.lib.util.HolonomicFeedforward;
import org.frogforce503.lib.util.SwerveTranslationalPID;
import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2021.Robot;
import org.frogforce503.robot2021.RobotState;
import org.frogforce503.robot2021.RobotState.GameState;
import org.frogforce503.robot2021.loops.FroggyPoseController;
import org.frogforce503.robot2021.subsystems.Subsystem;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem {

    private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.2, 0.01, 0.0);
    private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
            new DrivetrainFeedforwardConstants(1.0 / (12.0 * 12.0), 0.0, 0.0));
    // Instance declaration
    private static Swerve instance = null;
    public final double DRIVER_INPUT_TOLERANCE_TRANSLATION = 0.05;
    public final double DRIVER_INPUT_TOLERANCE_ROTATION = 0.05;
    private final double kLengthComponent;
    private final double kWidthComponent;
    private final FrogPIDF visionAnglePID = Robot.bot.visionAnglePID;
    private final FrogPIDF stabilizationPID = Robot.bot.stabilizationPID;
    private final FrogPIDF snappingPID = Robot.bot.snappingPID;
    private final SwerveTranslationalPID visionTranslationPID = Robot.bot.visionTranslationalPID;
    private final Translation2d ZERO_VECTOR = new Translation2d(0.0, 0.0);
    private final List<SwerveModule> modules;
    private final PIDController xController = new PIDController(0.008 * 3, 0.0, 0.002);
    private final PIDController yController = new PIDController(0.008 * 3, 0.0, 0.002);
    // Module declaration
    public SwerveModule backRight, backLeft, frontRight, frontLeft;
    private double visionDistanceSetpoint = 100;
    private boolean visionDistanceOnTarget = false;
    private boolean visionDistanceNearTarget = false;
    // Teleop driving variables
    private Translation2d translationalVector = new Translation2d();
    private double rotationalInput = 0;
    private double testTheta = 0;
    private Translation2d centerOfRotation = new Translation2d();
    private double stabilizationUpdateTime = 0.0;
    private boolean lowPower;
    private boolean swerveCoasted = false;
    private volatile double currentPathSetpoint = 0.0;
    private volatile double pathAngleUpdateDelay = 0.0;
    private volatile double currentSnappingSetpoint = 0.0;
    private volatile double currentStablizationSetpoint = 0.0;
    private volatile double lastVisionUpdate = -1.0;
    private volatile double currentVisionLimelightBasedControlCommand = 0.0;
    private volatile double requestedDriverHeadingInput = 0.0;
    private volatile double lastPathAngleUpdate = 0.0;
    private HolonomicPurePursuitTrajectoryFollower follower;
    private volatile Translation2d requestedDriverTranslation = new Translation2d(0.0, 0.0);
    private volatile Translation2d requestedAutonTranslation = new Translation2d(0.0, 0.0);
    private TranslationControlMode translationMode = TranslationControlMode.DRIVER_CONTROLLED;
    private HeadingControlMode headingMode = HeadingControlMode.DRIVER_CONTROLLED;
    private boolean fieldCentric = true;
    private boolean lockedOnPowerPort = false;
    private volatile boolean autonHeadingOnTarget = false;
    private final PIDController visionDistancePID = new PIDController(1.0 / 100.0, 0.000, 0.0);
    private final SlewRateLimiter visionRateLimiter = new SlewRateLimiter(3.0);
    // AutoNav trajectory follower
    private volatile edu.wpi.first.wpilibj.trajectory.Trajectory currentTrajectory = new edu.wpi.first.wpilibj.trajectory.Trajectory();
    private volatile HolonomicTrajectory currentHolonomicTrajectory = new HolonomicTrajectory();

    // Set track width to maximize speed around curves to 140in/sec
    // Velocity and accel = 164
    private double trajectoryInitTime = 0;

    // Field graphing instance
    private final Field2d fieldMap = new Field2d();
    private Pose2d metricPose;

    // Constructor
    public Swerve() {
        try {
            this.backRight = Util.readSwerveJSON(Robot.bot.backRightName);
            this.backLeft = Util.readSwerveJSON(Robot.bot.backLeftName);
            this.frontRight = Util.readSwerveJSON(Robot.bot.frontRightName);
            this.frontLeft = Util.readSwerveJSON(Robot.bot.frontLeftName);
        } catch (final Exception e) {
            e.printStackTrace();
        }
        backRight.setID("br");
        backLeft.setID("bl");
        frontRight.setID("fr");
        frontLeft.setID("fl");
        modules = Arrays.asList(backRight, backLeft, frontLeft, frontRight);

        final double width = Robot.bot.kWheelbaseWidth, length = Robot.bot.kWheelbaseLength;
        final double radius = Math.hypot(width, length);
        FrogPIDF autuonomousAnglePID = Robot.bot.autuonomousAnglePID;
        autuonomousAnglePID.setTolerance(2.0);
        visionAnglePID.setTolerance(2.0);
        stabilizationPID.setTolerance(2.0);
        snappingPID.setTolerance(2.0);
        kLengthComponent = length / radius;
        kWidthComponent = width / radius;

        SmartDashboard.putData("Field", fieldMap);
    }

    public static Swerve getInstance() {
        if (instance == null)
            instance = new Swerve();
        return instance;
    }

    public static SwerveModuleState[] getRawModuleStates(Translation2d desiredtranslation, double rotation,
            Translation2d[] moduleRadaii) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        double maxMagnitude = 0.0;
        for (int i = 0; i < states.length; i++) {
            Translation2d radiusVector = moduleRadaii[i];
            Translation2d rotationVector = radiusVector.times(rotation);
            Translation2d wheelVector = desiredtranslation.plus(rotationVector);
            double wheelSpeed = wheelVector.getNorm();
            maxMagnitude = wheelSpeed > maxMagnitude ? wheelSpeed : maxMagnitude;
            double wheelAngleRaw = Math.toDegrees(Math.atan2(wheelVector.getY(), wheelVector.getX()));
            double unitCircleWheelAngle = wheelAngleRaw;
            if (wheelVector.getX() > 0 && wheelVector.getY() < 0) {
                unitCircleWheelAngle += 360;
            }
            double zeroForwardWheelAngle = 90 - unitCircleWheelAngle;
            zeroForwardWheelAngle = Util.boundAngle0to360Degrees(zeroForwardWheelAngle);
            states[i] = new SwerveModuleState();
            states[i].angle = Rotation2d.fromDegrees(zeroForwardWheelAngle);
            states[i].speedMetersPerSecond = wheelSpeed;
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < states.length; i++) {
                states[i].speedMetersPerSecond /= maxMagnitude;
            }
        }
        return states;
    }

    // Constructor
    public void setLowPower(final boolean lowPower) {
        this.lowPower = lowPower;
    }

    private boolean getLowPowerRequested() {
        return lowPower;
    }

    public Translation2d getRequestedAutonTranslation() {
        return requestedAutonTranslation;
    }

    public void setRequestedAutonTranslation(final Translation2d requestedAutonTranslation) {
        this.requestedAutonTranslation = requestedAutonTranslation;
    }

    public Translation2d getRequestedDriverTranslation() {
        return requestedDriverTranslation;
    }

    public void setRequestedDriverTranslation(final Translation2d requestedDriverTranslation) {
        this.requestedDriverTranslation = requestedDriverTranslation;
    }

    public TranslationControlMode getTranslationMode() {
        return translationMode;
    }

    public void setTranslationMode(final TranslationControlMode mode) {
        this.translationMode = mode;
    }

    public HeadingControlMode getHeadingMode() {
        return headingMode;
    }

    public void setHeadingMode(final HeadingControlMode mode) {
        this.headingMode = mode;
    }

    /**
     * @return backRight, backLeft, frontLeft, frontRight
     */
    public List<SwerveModule> getModules() {
        return modules;
    }

    public double[][] getWheelComponentVelocities() {
        final double[][] returner = { { frontRight.getXComponentVelocity() }, { frontRight.getYComponentVelocity() },
                { frontLeft.getXComponentVelocity() }, { frontLeft.getYComponentVelocity() },
                { backLeft.getXComponentVelocity() }, { backLeft.getYComponentVelocity() },
                { backRight.getXComponentVelocity() }, { backRight.getYComponentVelocity() } };
        return returner;

    }

    /**
     * @return the fieldCentric
     */
    public boolean isFieldCentric() {
        return fieldCentric;
    }

    /**
     * @param fieldCentric
     */
    public void setFieldCentric(final boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public void snapForward() {
        backRight.drive(503.0, 0.0);
        backLeft.drive(503.0, 0.0);
        frontRight.drive(503.0, 0.0);
        frontLeft.drive(503.0, 0.0);
    }

    public void initializeSwerveStabilization() {
        this.currentStablizationSetpoint = RobotState.getInstance().getCurrentTheta();
    }

    public void reverseSwerveStabilization() {
        this.currentStablizationSetpoint = 180;
    }

    public void zeroSwerveStabilization() {
        this.currentStablizationSetpoint = 0;
    }

    private Translation2d[] getModuleRadaii() {
        Translation2d[] radaii = { Robot.bot.kBackRightRadius, Robot.bot.kBackLeftRadius, Robot.bot.kFrontLeftRadius,
                Robot.bot.kFrontRightRadius };
        return radaii;
    }

    public void vectorDrive(Translation2d translationVector, double rcw, final boolean lowPower) {
        if (lowPower) {
            translationVector = translationVector.times(0.3);
        }
        rcw *= lowPower ? 0.5 : 1.0;

        if (fieldCentric) {
            double str = translationVector.getX();
            double fwd = translationVector.getY();
            final double angle = Math.toRadians(RobotState.getInstance().getCurrentTheta());
            final double temp = fwd * Math.cos(angle) + str * Math.sin(angle);
            str = -fwd * Math.sin(angle) + str * Math.cos(angle);
            fwd = temp;
            translationVector = new Translation2d(str, fwd);
        }

        rotationalInput = rcw;
        SwerveModuleState[] states = getRawModuleStates(translationVector, rotationalInput, getModuleRadaii());

        for (int i = 0; i < states.length; i++) {
            if (shouldReverse(states[i].angle.getDegrees(), modules.get(i).getTurnEncoderPosition())) {
                states[i].angle = Rotation2d.fromDegrees(states[i].angle.getDegrees() + 180);
                states[i].speedMetersPerSecond *= -1;
            }
        }

        backRight.drive(states[0]);
        backLeft.drive(states[1]);
        frontLeft.drive(states[2]);
        frontRight.drive(states[3]);

    }

    /**
     * @param str      Strafe power (-1 to 1)
     * @param fwd      Forward power (-1 to 1)
     * @param rcw      Rotation power (-1 to 1)
     * @param lowPower Boolean for low power mode
     */
    public void drive(double str, double fwd, double rcw, final boolean lowPower) {
        translationalVector = new Translation2d(str, fwd);
        str *= (lowPower ? 0.3 : 1.0) * Robot.bot.requestDriveReversed;
        fwd *= (lowPower ? 0.5 : 1.0) * Robot.bot.requestDriveReversed;
        rcw *= lowPower ? 0.5 : 1.0;

        if (fieldCentric) {
            final double angle = Math.toRadians(RobotState.getInstance().getCurrentTheta());
            final double temp = fwd * Math.cos(angle) + str * Math.sin(angle);
            str = -fwd * Math.sin(angle) + str * Math.cos(angle);
            fwd = temp;
        }

        rotationalInput = rcw;

        final double a = str - rcw * kLengthComponent;
        final double b = str + rcw * kLengthComponent;
        final double c = fwd - rcw * kWidthComponent;
        final double d = fwd + rcw * kWidthComponent;

        double backRightSpeed = Math.hypot(a, c);
        double backLeftSpeed = Math.hypot(a, d);
        double frontRightSpeed = Math.hypot(b, c);
        double frontLeftSpeed = Math.hypot(b, d);

        double backRightAngle = (Math.atan2(a, c) * 180 / Math.PI);
        double backLeftAngle = (Math.atan2(a, d) * 180 / Math.PI);
        double frontRightAngle = (Math.atan2(b, c) * 180 / Math.PI);
        double frontLeftAngle = (Math.atan2(b, d) * 180 / Math.PI);

        // normalize wheel speeds
        double max = frontRightSpeed;
        if (frontLeftSpeed > max) {
            max = frontLeftSpeed;
        }
        if (backLeftSpeed > max) {
            max = backLeftSpeed;
        }
        if (backRightSpeed > max) {
            max = backRightSpeed;
        }
        if (max > 1.0) {
            frontRightSpeed /= max;
            frontLeftSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        // ensures no wheel rotates more than 90 degrees at a time.
        if (shouldReverse(backRightAngle, backRight.getTurnEncoderPositioninDegrees())) {
            backRightAngle += 180;
            backRightSpeed *= -1;
        }
        if (shouldReverse(backLeftAngle, backLeft.getTurnEncoderPositioninDegrees())) {
            backLeftAngle += 180;
            backLeftSpeed *= -1;
        }
        if (shouldReverse(frontRightAngle, frontRight.getTurnEncoderPositioninDegrees())) {
            frontRightAngle += 180;
            frontRightSpeed *= -1;
        }
        if (shouldReverse(frontLeftAngle, frontLeft.getTurnEncoderPositioninDegrees())) {
            frontLeftAngle += 180;
            frontLeftSpeed *= -1;
        }

        // Send speeds and angles to the drive motors
        if (translationMode != TranslationControlMode.HOLONOMIC_AUTO) {
            backRight.drive(backRightSpeed, backRightAngle);
            backLeft.drive(backLeftSpeed, backLeftAngle);
            frontRight.drive(frontRightSpeed, frontRightAngle);
            frontLeft.drive(frontLeftSpeed, frontLeftAngle);
        } else {
            // ? is max theoretical velocity
            backRight.driveWithVelocity(backRightSpeed * 170, backRightAngle);
            backLeft.driveWithVelocity(backLeftSpeed * 170, backLeftAngle);
            frontRight.driveWithVelocity(frontRightSpeed * 170, frontRightAngle);
            frontLeft.driveWithVelocity(frontLeftSpeed * 170, frontLeftAngle);
            // System.out.println("backLeft speed :" + backLeftSpeed);
        }
    }

    private void drive(final Translation2d translationVector, final double rotIn, final boolean lowPower) {
        // translationVector = translationVector.normalize();
        final double str = translationVector.getX();
        final double fwd = translationVector.getY();
        drive(str, fwd, rotIn, lowPower);
    }

    private void defensePosition() {
        if (translationMode == TranslationControlMode.ANCHOR) {
            final double backRightSpeed = 503.0, backLeftSpeed = 503.0, frontRightSpeed = 503.0, frontLeftSpeed = 503.0,
                    backRightAngle = -45, backLeftAngle = 45, frontLeftAngle = -45, frontRightAngle = 45;
            backRight.drive(backRightSpeed, backRightAngle);
            backLeft.drive(backLeftSpeed, backLeftAngle);
            frontRight.drive(frontRightSpeed, frontRightAngle);
            frontLeft.drive(frontLeftSpeed, frontLeftAngle);
        }
    }

    public Translation2d getCurrentTranslationVector() {
        return translationalVector;
    }

    /**
     * Master control loop for the swerve drive. Call this in a loop repeadetly.
     * Determines rotational power using rotation state machine and independently
     * determines translational input based on a seperate state machine. Desired
     * states should be updated using the update methods.
     */
    public void control() {

        Translation2d requestedTranslation;
        SmartDashboard.putString("Control State", translationMode.toString());
        SmartDashboard.putString("Game State", RobotState.getInstance().getGameState().toString());
        switch (translationMode) {
            case DRIVER_CONTROLLED:
                requestedTranslation = requestedDriverTranslation;
                break;
            case AUTONOMOUS_CONTROL:
                if (RobotState.getInstance().getGameState() == GameState.AUTON) {
                    requestedTranslation = requestedAutonTranslation;
                    SmartDashboard.putBoolean("auton state error", false);
                } else {
                    requestedAutonTranslation = ZERO_VECTOR;
                    System.err.println("AUTON RUNNING IN TELEOP");
                    SmartDashboard.putBoolean("auton state error", true);
                    translationMode = TranslationControlMode.DRIVER_CONTROLLED;
                    requestedTranslation = ZERO_VECTOR;
                }
                break;
            case AUTONOMOUS:
                if (RobotState.getInstance().getGameState() == GameState.AUTON) {
                    double trajectoryTime = Timer.getFPGATimestamp() - trajectoryInitTime;
                    edu.wpi.first.wpilibj.trajectory.Trajectory.State setpointState = currentTrajectory
                            .sample(trajectoryTime);
                    edu.wpi.first.wpilibj.trajectory.Trajectory.State nextState = currentTrajectory
                            .sample(trajectoryTime + 0.005);
                    Translation2d translationSetpoint = setpointState.poseMeters.getTranslation();
                    double velocitySetpoint = setpointState.velocityMetersPerSecond;
                    double nextVelocitySetpoint = nextState.velocityMetersPerSecond;
                    Rotation2d thetaSetpoint = setpointState.poseMeters.getRotation();
                    Translation2d currentTranslation = RobotState.getInstance().getCurrentPose().getTranslation();

                    double xVelocity = velocitySetpoint * thetaSetpoint.getCos();
                    double yVelocity = velocitySetpoint * thetaSetpoint.getSin();

                    double xAcceleration = (nextVelocitySetpoint * thetaSetpoint.getCos() - xVelocity) / 0.005;
                    double yAcceleration = (nextVelocitySetpoint * thetaSetpoint.getSin() - yVelocity) / 0.005;

                    double kxV = 1. / 180.; // WRONG CONSTANTS
                    double kyV = 1. / 180.;
                    double kA = 0.0 / 164.0;

                    requestedAutonTranslation = new Translation2d(
                            -yController.calculate(currentTranslation.getY(), translationSetpoint.getY())
                                    - yVelocity * kyV - yAcceleration * kA,
                            xController.calculate(currentTranslation.getX(), translationSetpoint.getX())
                                    + xVelocity * kxV + xAcceleration * kA);

                    requestedTranslation = requestedAutonTranslation;

                    // Swerve.getInstance().updateDriverTranslation(mTranslationOutput);
                    SmartDashboard.putNumber("Goal X", translationSetpoint.getX());
                    SmartDashboard.putNumber("Goal Y", translationSetpoint.getY());
                    SmartDashboard.putNumber("CURRENT X", currentTranslation.getX());
                    SmartDashboard.putNumber("CURRENT Y", currentTranslation.getY());
                    SmartDashboard.putNumber("ERROR X", translationSetpoint.getX() - currentTranslation.getX());
                    SmartDashboard.putNumber("ERROR Y", translationSetpoint.getY() - currentTranslation.getY());

                } else {
                    requestedAutonTranslation = ZERO_VECTOR;
                    System.err.println("AUTON RUNNING IN TELEOP");
                    SmartDashboard.putBoolean("auton state error", true);
                    translationMode = TranslationControlMode.DRIVER_CONTROLLED;
                    requestedTranslation = ZERO_VECTOR;
                }
                break;
            case HOLONOMIC_AUTO:
                if (RobotState.getInstance().getGameState() == GameState.AUTON) {
                    double trajectoryTime = Timer.getFPGATimestamp() - trajectoryInitTime;
                    HolonomicTrajectory.State setpointState = currentHolonomicTrajectory.sample(trajectoryTime);
                    HolonomicTrajectory.State nextState = currentHolonomicTrajectory.sample(trajectoryTime + 0.005);
                    Translation2d translationSetpoint = setpointState.poseMeters.getTranslation();
                    double velocitySetpoint = setpointState.velocityMetersPerSecond;
                    double nextVelocitySetpoint = nextState.velocityMetersPerSecond;
                    Rotation2d thetaSetpoint = setpointState.poseMeters.getRotation();
                    Translation2d currentTranslation = RobotState.getInstance().getCurrentPose().getTranslation();

                    double xVelocity = velocitySetpoint * thetaSetpoint.getCos();
                    double yVelocity = velocitySetpoint * thetaSetpoint.getSin();

                    double xAcceleration = (nextVelocitySetpoint * thetaSetpoint.getCos() - xVelocity) / 0.005;
                    double yAcceleration = (nextVelocitySetpoint * thetaSetpoint.getSin() - yVelocity) / 0.005;

                    double kxV = 1.0 / 170.0;
                    double kyV = 1.0 / 170.0;
                    double kA = 0.0 / 164.0;

                    double xGain = -yVelocity * kyV
                            - yController.calculate(currentTranslation.getY(), translationSetpoint.getY());
                    double yGain = xVelocity * kxV
                            + xController.calculate(currentTranslation.getX(), translationSetpoint.getX());

                    // @formatter:off
                    requestedAutonTranslation = new Translation2d(
                       xGain,
                       yGain
                    );
                    // @formatter:on
                    requestedTranslation = requestedAutonTranslation;
                    currentSnappingSetpoint = setpointState.holonomicHeading;

                    // System.out.println("xGain: " + xGain + " yGain: " + yGain);

                    // System.out.println("Target Heading: " + setpointState.holonomicHeading);

                    // Swerve.getInstance().updateDriverTranslation(mTranslationOutput);
                    SmartDashboard.putNumber("Goal X", translationSetpoint.getX());
                    SmartDashboard.putNumber("Goal Y", translationSetpoint.getY());
                    SmartDashboard.putNumber("CURRENT X", currentTranslation.getX());
                    SmartDashboard.putNumber("CURRENT Y", currentTranslation.getY());
                    // System.out.println("Setpoint: " + translationSetpoint.getX() + "Encoder : "
                    // + backLeft.getDriveEncoderClicks());

                    SmartDashboard.putNumber("ERROR X", translationSetpoint.getX() - currentTranslation.getX());
                    SmartDashboard.putNumber("ERROR Y", translationSetpoint.getY() - currentTranslation.getY());
                    SmartDashboard.putNumber("Acceleration Gain", xAcceleration * kA);

                } else {
                    requestedAutonTranslation = ZERO_VECTOR;
                    System.err.println("AUTON RUNNING IN TELEOP");
                    SmartDashboard.putBoolean("auton state error", true);
                    translationMode = TranslationControlMode.DRIVER_CONTROLLED;
                    requestedTranslation = ZERO_VECTOR;
                }
                break;
            case VISION:
                setFieldCentric(false);

                boolean targetVisible = LimelightProcessor.getInstance().getTV() == 1.0;

                double yVisionGain = visionRateLimiter.calculate(visionDistancePID
                        .calculate(LimelightProcessor.getInstance().distFromTarget(), visionDistanceSetpoint));

                yVisionGain = Math.max(Math.min(yVisionGain, 0.75), -0.75);

                requestedTranslation = new Translation2d(0.0, targetVisible ? yVisionGain : 0.1);

                visionDistanceOnTarget = Math.abs(visionDistancePID.getPositionError()) < 2.0
                        && Math.abs(visionDistancePID.getVelocityError()) < 6.0;

                visionDistanceNearTarget = Math.abs(visionDistancePID.getPositionError()) < 20.0;
                break;
            case ANCHOR:
                // locks all wheels in the following fashion to become effectively immovable
                /*
                 * \ /
                 *
                 *
                 * / \
                 */
                requestedTranslation = new Translation2d(0.0, 0.0);
                defensePosition();
                return;
            default:
                requestedTranslation = ZERO_VECTOR;
                break;
        }
        double rotationalInput = getRequestedRotation();

        SmartDashboard.putNumber("BackLeft VelError", backLeft.getVelocityError());
        if (RobotState.getInstance().getGameState() != GameState.TEST)
            drive(requestedTranslation, rotationalInput, getLowPowerRequested());
    }

    /***
     * The following code is a state machine to control the swerve drive's
     * rotational power at every point of the match. runs automatically from the
     * Swerve.control() method. Update desired states through the update methods
     * belowl;
     *
     * @return rotational swerve output
     ****/
    private synchronized double getRequestedRotation() {

        final double curHeading = RobotState.getInstance().getCurrentTheta();
        stabilizationPID.setSetpoint(currentStablizationSetpoint);
        double stabilizationLag = 0.5;
        double LIMELIGHT_NO_SIGNAL_KILL_THRESHHOLD = 0.000001;
        switch (headingMode) {
            case DRIVER_CONTROLLED:
                // set rotational output based on driver input
                currentStablizationSetpoint = curHeading;
                return requestedDriverHeadingInput;
            case VISION_GYRO:
                // set rotational output based on a desired gyro angle as determined by gyro
                currentStablizationSetpoint = RobotState.getInstance().getCurrentTheta();
                if (Timer.getFPGATimestamp() - lastVisionUpdate > LIMELIGHT_NO_SIGNAL_KILL_THRESHHOLD) {
                    headingMode = HeadingControlMode.STABILIZING;
                } else {
                    return visionAnglePID.calculateOutput(curHeading, true);
                }
            case VISION_LIMELIGHT:
                // set rotational output to power value as calculate by limelight tx value
                System.out.println("IN VISION LOCK");
                currentStablizationSetpoint = curHeading;
                return currentVisionLimelightBasedControlCommand;
            case AUTONOMOUS_TARGET:
                autonHeadingOnTarget = false;
                if ((Timer.getFPGATimestamp() - lastPathAngleUpdate > pathAngleUpdateDelay)) {
                    currentSnappingSetpoint = currentPathSetpoint;
                    snappingPID.setSetpoint(currentSnappingSetpoint);

                    if (snappingPID.onTarget()) {
                        headingMode = HeadingControlMode.STABILIZING;
                        currentStablizationSetpoint = currentSnappingSetpoint;
                        autonHeadingOnTarget = true;
                    }
                    double rcw = snappingPID.calculateOutput(curHeading, true);
                    return rcw * 0.3;
                }
                // auton not yet ready to snap to angle
                else {
                    return stabilizationPID.calculateOutput(curHeading, true);
                }
            case SNAPPING:
                // Snaps to a desired teleop setpoint until on target
                snappingPID.setSetpoint(currentSnappingSetpoint);
                // if (snappingPID.onTarget()) {
                // headingMode = HeadingControlMode.STABILIZING;
                // currentStablizationSetpoint = RobotState.getInstance().getCurrentTheta();
                // }
                double snappingGain = 0.3 * snappingPID.calculateOutput(curHeading, true);
                // System.out.println("snapping gain: " + snappingGain);
                return snappingGain;
            case STABILIZING:
                // only stabilize once robot is settled after a turn. possible improvement: make
                // it based on robot angular acceleration instead of input
                if (Timer.getFPGATimestamp() - stabilizationUpdateTime > stabilizationLag) {
                    return stabilizationPID.calculateOutput(curHeading, true);
                } else {
                    currentStablizationSetpoint = curHeading;
                    return 0.0;
                }
            default:
                System.err.print("ERROR STATE = " + headingMode);
                return 0.0;
        }
    }

    /****************************************************************************
     * * * Below are all of the methods to send desired states/inputs to the swerve*
     * * *
     ****************************************************************************/

    public synchronized void updateVisionAngularSetpoint(final double visionSetpoint) {
        visionAnglePID.setSetpoint(visionSetpoint);
        headingMode = HeadingControlMode.VISION_GYRO;
        this.lastVisionUpdate = Timer.getFPGATimestamp();
    }

    public synchronized void updateVisionLimelightAngleInput(final double visionInput) {
        this.currentVisionLimelightBasedControlCommand = visionInput;
        headingMode = HeadingControlMode.VISION_LIMELIGHT;
        this.lastVisionUpdate = Timer.getFPGATimestamp();
    }

    public synchronized void updatePathAngularSetpoint(final double pathAngularSetpoint, final double delay,
            final Translation2d poseToTurn, final double timeoutOnPose) {
        this.currentPathSetpoint = pathAngularSetpoint;
        this.pathAngleUpdateDelay = delay;
        this.lastPathAngleUpdate = Timer.getFPGATimestamp();
        headingMode = HeadingControlMode.AUTONOMOUS_TARGET;
    }

    public synchronized void snapToAngle(final double angle) {
        this.currentSnappingSetpoint = angle;
        headingMode = HeadingControlMode.SNAPPING;
    }

    public synchronized void enableAnchorMode() {
        translationMode = TranslationControlMode.ANCHOR;
    }

    public synchronized void killAnchorMode() {
        if (translationMode == TranslationControlMode.ANCHOR) {
            translationMode = TranslationControlMode.DRIVER_CONTROLLED;
        }
    }

    public synchronized void updateVisionTranslation(final Translation2d visionTranslation,
            final VisionTranslationControlMode mode) {
        translationMode = TranslationControlMode.VISION;
        Translation2d requestedVisionTranslation = ZERO_VECTOR;
        if (mode == VisionTranslationControlMode.LIMELIGHT) {
            requestedVisionTranslation = visionTranslation;
            SmartDashboard.putString("Vision translation", visionTranslation.toString());
        } else {
            visionTranslationPID.updateSetpoint(visionTranslation);
            if (visionTranslationPID.onTarget()) {
                translationMode = TranslationControlMode.DRIVER_CONTROLLED;
            } else {
                requestedVisionTranslation = visionTranslationPID
                        .getOutput(RobotState.getInstance().getCurrentPose().getTranslation());
            }
        }
    }

    public synchronized void driveToTargetDistance(final double distance) {
        translationMode = TranslationControlMode.VISION;
        this.visionDistanceSetpoint = distance;
        setFieldCentric(false);

    }

    public synchronized boolean getVisionDistanceOnTarget() {
        return this.visionDistanceOnTarget;
    }

    public synchronized boolean getVisionDistanceNearTarget() {
        return this.visionDistanceNearTarget;
    }

    public synchronized boolean isHolonomicFollowerFinished() {
        return Timer.getFPGATimestamp() - trajectoryInitTime >= currentHolonomicTrajectory.getTotalTimeSeconds()
                && requestedAutonTranslation.getNorm() < 0.10;
    }

    public synchronized void updateDriverTranslation(final Translation2d driveTranslation) {
        requestedDriverTranslation = driveTranslation;
        translationMode = TranslationControlMode.DRIVER_CONTROLLED;
    }

    public synchronized void noDriverTranslationInput() {
        requestedDriverTranslation = ZERO_VECTOR;
    }

    public synchronized void noDriverHeadingInput() {
        requestedDriverHeadingInput = 0.0;
        headingMode = HeadingControlMode.STABILIZING;
    }

    public synchronized void updateDriverHeadingInput(final double rcw) {
        requestedDriverHeadingInput = rcw;
        headingMode = HeadingControlMode.DRIVER_CONTROLLED;
        stabilizationUpdateTime = Timer.getFPGATimestamp();
    }

    public synchronized void setAutonomousTrajectory(Trajectory trajectory) {
        follower = new HolonomicPurePursuitTrajectoryFollower(10, 10, FOLLOWER_FEEDFORWARD_CONSTANTS,
                FOLLOWER_ROTATION_CONSTANTS);
        follower.follow(trajectory);
        translationMode = TranslationControlMode.AUTONOMOUS_CONTROL;
    }

    public synchronized void setHolonomicFollowerTrajectory(edu.wpi.first.wpilibj.trajectory.Trajectory trajectory,
            boolean resetPose) {
        this.currentTrajectory = trajectory;

        xController.reset();
        yController.reset();

        Swerve.getInstance().noDriverHeadingInput();

        this.trajectoryInitTime = Timer.getFPGATimestamp();
        edu.wpi.first.wpilibj.trajectory.Trajectory.State initialState = currentTrajectory.sample(0.0);
        Pose2d initialPose = initialState.poseMeters;
        if (resetPose) {

            FroggyPoseController.resetPose(new Pose2d(initialPose.getTranslation(), new Rotation2d()));

        }

        translationMode = TranslationControlMode.AUTONOMOUS;
    }

    public synchronized void setHolonomicHeadingTrajectory(HolonomicTrajectory trajectory, boolean resetPose) {
        this.currentHolonomicTrajectory = trajectory;

        xController.reset();
        yController.reset();

        // Swerve.getInstance().noDriverHeadingInput();

        this.trajectoryInitTime = Timer.getFPGATimestamp();
        HolonomicTrajectory.State initialState = currentHolonomicTrajectory.sample(0.0);
        Pose2d initialPose = initialState.poseMeters;
        if (resetPose) {

            FroggyPoseController.resetPose(new Pose2d(initialPose.getTranslation(), new Rotation2d()));

        }

        translationMode = TranslationControlMode.HOLONOMIC_AUTO;
        headingMode = HeadingControlMode.SNAPPING;
    }

    public synchronized void setAutonomousTrajectory(Trajectory trajectory, double lookAhead) {
        follower = new HolonomicPurePursuitTrajectoryFollower(lookAhead, lookAhead, FOLLOWER_FEEDFORWARD_CONSTANTS,
                FOLLOWER_ROTATION_CONSTANTS);
        follower.follow(trajectory);
        translationMode = TranslationControlMode.AUTONOMOUS_CONTROL;
    }

    public synchronized void updateAutonomousTrajectoryFollowerOutput(final Pose2d frogPose) {
        final Vector2 robotTranslation = new Vector2(frogPose.getTranslation().getX(),
                -frogPose.getTranslation().getY());
        final Rotation2 robotRotation = Rotation2.fromDegrees(frogPose.getRotation().getDegrees());
        final RigidTransform2 pose = new RigidTransform2(robotTranslation, robotRotation);
        final Optional<HolonomicDriveSignal> signal = follower.update(pose, new Vector2(0.0, 0.0), 0.0,
                Timer.getFPGATimestamp(), 0.05);
        final Vector2 translation = signal.get().getTranslation();

        final Translation2d frogTranslation = new Translation2d(translation.y, translation.x);

        this.requestedAutonTranslation = frogTranslation;
        if (follower.isFinished()) {
            this.requestedAutonTranslation = ZERO_VECTOR;
            this.requestedDriverTranslation = ZERO_VECTOR;
            translationMode = TranslationControlMode.DRIVER_CONTROLLED;
            return;
        }
        translationMode = TranslationControlMode.AUTONOMOUS_CONTROL;
    }

    public synchronized void directUpdateAutonomousFollowerOutputTranslation(
            final Translation2d requestedAutonTranslation) {
        this.requestedAutonTranslation = requestedAutonTranslation;
        translationMode = TranslationControlMode.AUTONOMOUS_CONTROL;
    }

    public synchronized void killVisionTranslationInput() {
        translationMode = TranslationControlMode.DRIVER_CONTROLLED;
    }

    public synchronized void killVisionAngularInput() {
        headingMode = HeadingControlMode.DRIVER_CONTROLLED;
    }

    public synchronized void killAutonTranslationInput() {
        requestedDriverTranslation = ZERO_VECTOR;
        translationMode = TranslationControlMode.DRIVER_CONTROLLED;
        requestedAutonTranslation = ZERO_VECTOR;
    }

    public synchronized boolean isTrajectoryFinished() {
        return follower.isFinished();
    }

    public synchronized boolean isAutonHeadingOnTarget() {
        return autonHeadingOnTarget;
    }

    /**
     * @param goalAngle    Target Angle through drive vectors
     * @param currentAngle Current Angle of swerve module
     * @return if the module phase should be inverted
     */
    private boolean shouldReverse(final double goalAngle, final double currentAngle) {
        return Util.alternateShouldReverse(goalAngle, currentAngle);
    }

    public void testModules(double speed) {
        // testTheta = Math.PI / 2;
        this.drive(Math.cos(testTheta) * speed, Math.sin(testTheta) * speed, 0.0, false);
        testTheta += 0.01;
    }

    public void spinTest(double speed, double angle) {
        backLeft.drive(0.5, angle);
        backRight.drive(0.5, angle);
        frontLeft.drive(0.5, angle);
        frontRight.drive(0.5, angle);
    }

    public Translation2d getCenterOfRotation() {
        return this.centerOfRotation;
    }

    public void setCenterOfRotation(final Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
    }

    public void setCenterOfRotation(final double x, final double y) {
        setCenterOfRotation(new Translation2d(x, y));
    }

    public void setDriveBrakeMode() {
        modules.forEach(SwerveModule::brakeDrive);
    }

    public void setDriveCoastMode() {
        modules.forEach(SwerveModule::coastDrive);
    }

    public void setAzimuthBrakeMode() {
        modules.forEach(SwerveModule::brakeAzimuth);
    }

    public void setAzimuthCoastMode() {
        modules.forEach(SwerveModule::coastAzimuth);
    }

    public void brakeAll() {
        swerveCoasted = false;
        setDriveBrakeMode();
        setAzimuthBrakeMode();
    }

    public void coastAll() {
        swerveCoasted = true;
        setDriveCoastMode();
        setAzimuthCoastMode();
    }

    public boolean getCoasted() {
        return swerveCoasted;
    }

    public void resetDriveEncoder() {
        modules.forEach((mod) -> mod.resetDriveEncoder());
    }

    @Override
    public void setCurrentLimit(final int limit) {
        modules.forEach((mod) -> mod.setDriveMotorCurrentLimit(limit));
    }

    @Override
    public void outputTelemetry() {
        modules.forEach(mod -> mod.outputTelemetry());
        SmartDashboard.putBoolean("Field Centric: ", isFieldCentric());
        SmartDashboard.putNumber("HEADING SETPOINT: ", currentPathSetpoint);

        // Convert pose from inches to Meters for graph
        Pose2d currPose = RobotState.getInstance().getCurrentPose();
        double inToM = 0.0254;
        metricPose = new Pose2d(currPose.getX() * inToM, currPose.getY() * inToM, currPose.getRotation());

        fieldMap.setRobotPose(metricPose);
    }

    public HeadingControlMode getCurrentHeadingMode() {
        return headingMode;
    }

    @Override
    public void stop() {
        modules.forEach((m) -> m.stop());
    }

    @Override
    public void zeroSensors() {
        resetDriveEncoder();
    }

    @Override
    public void onStart(final double timestamp) {
    }

    @Override
    public void onLoop(final double timestamp) {
    }

    @Override
    public void onStop(final double timestamp) {
    }

    public enum TranslationControlMode {
        DRIVER_CONTROLLED, AUTONOMOUS_CONTROL, ANCHOR, VISION, AUTONOMOUS, HOLONOMIC_AUTO
    }

    public enum HeadingControlMode {
        VISION_GYRO, VISION_LIMELIGHT, AUTONOMOUS_TARGET, SNAPPING, DRIVER_CONTROLLED, STABILIZING, LEGACY_ALIGN
    }

    public enum VisionTranslationControlMode {
        POSE, LIMELIGHT
    }
}