package org.frogforce503.robot2021.subsystems.vision;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2021.Constants;
import org.frogforce503.robot2021.Robot;
import org.frogforce503.robot2021.RobotState;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.subsystems.swerve.Swerve.VisionTranslationControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightProcessor {

    private static LimelightProcessor instance = null;

    private final NetworkTable table, floorTable;
    private final NetworkTableEntry ledMode, pipeline, camMode, stream, ct, floorPipeline;
    private final List<NetworkTableEntry> combinedTarget, floorCombinedTarget;
    private final LinkedList<VisionData> collectedData = new LinkedList<VisionData>();
    private double tx, ty, ta, dist;
    // private LimelightState currentState = LimelightState.UP;
    // private TargetingState curTargetingState = TargetingState.OFF, previousState
    // = TargetingState.OFF;
    private Pipeline currentPipeline;

    private DoubleSolenoid limelightSolenoid;

    private LinearFilter txFilter = LinearFilter.singlePoleIIR(0.05, 0.01);
    private LinearFilter tyFilter = LinearFilter.singlePoleIIR(0.05, 0.01);

    /**
     * @param targetRange Target range, in inches
     */
    private double currentTargetRange;

    private LimelightProcessor() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

        ledMode = table.getEntry("ledMode");
        pipeline = table.getEntry("pipeline");
        camMode = table.getEntry("camMode");
        ct = table.getEntry("camtran");
        stream = table.getEntry("stream");

        combinedTarget = Arrays.asList(table.getEntry("tx"), table.getEntry("ty"), table.getEntry("ta"),
                table.getEntry("tv"));

        floorTable = NetworkTableInstance.getDefault().getTable("limelight-floor");
        floorPipeline = floorTable.getEntry("pipeline");
        floorCombinedTarget = Arrays.asList(floorTable.getEntry("tx"), floorTable.getEntry("ty"),
                floorTable.getEntry("ta"), floorTable.getEntry("tv"));

        setPipeline(Pipeline.DRIVER);

        if (!Robot.bot.hasMotorizedHood())
            limelightSolenoid = new DoubleSolenoid(6, 2);
    }

    public static LimelightProcessor getInstance() {
        return instance == null ? instance = new LimelightProcessor() : instance;
    }

    public void stopTargeting() {
        if (!Robot.bot.hasMotorizedHood())
            dropLimelight();

        setPipeline(Pipeline.DRIVER);
        this.tx = 0;
        this.ty = 0;
        this.ta = 0;
        this.dist = 0;
        SmartDashboard.putNumber("distance", 0);
        Swerve.getInstance().killVisionTranslationInput();
        Swerve.getInstance().killVisionAngularInput();
    }

    public void refreshStandard(Pipeline pipeline) {
        setPipeline(pipeline.standardID);
        this.tx = txFilter.calculate(getTX());
        this.ty = tyFilter.calculate(getTY());
        this.ta = getTA();
        this.dist = distFromTarget();
        SmartDashboard.putNumber("distance", dist);

        // if (getTV() == 1.0) {
        // if (pipeline == Pipeline.POWER_PORT && dist > 200 && isTargetVisible()) {
        // magnifyCurrentPipeline();
        // } else {
        // shrinkCurrentPipeline();
        // }
        // } else {
        // shrinkCurrentPipeline();
        // }
    }

    public void popLimelight() {
        // if (currentState.equals(LimelightState.DOWN)) {
        limelightSolenoid.set(Value.kForward);
        // System.out.println("Limelight solenoid set to up");
        // } else {
        // System.out.println("Limelight is already up");
        // }
        // LimelightProcessor.getInstance().setState(LimelightState.UP);
    }

    public void dropLimelight() {
        // if (currentState.equals(LimelightState.UP)) {
        SmartDashboard.putBoolean("LIMELIGHT", false);

        limelightSolenoid.set(Value.kReverse);
        setPipeline(Pipeline.DRIVER);
        // System.out.println("Limelight popped down");
        // }
        // LimelightProcessor.getInstance().setState(LimelightState.DOWN);
    }

    public void toggleLimelight() {
        limelightSolenoid.set(limelightSolenoid.get() == Value.kForward ? Value.kReverse : Value.kForward);
    }

    public void blink() {
        if (ledMode.getDouble(0) != 2) {
            ledMode.setNumber(2);
        }
    }

    public void ledOn(final boolean on) {
        if (ledMode.getDouble(1) != 0 && on) {
            ledMode.setNumber(0);
        } else if (ledMode.getDouble(0) != 1 && !on) {
            ledMode.setNumber(1);
        }
    }

    public void setDriverMode() {
        camMode.setNumber(1);
    }

    public void setVisionMode() {
        camMode.setNumber(0);
    }

    public void setStreamMode(final int id) {
        stream.setNumber(id);
    }

    /**
     * Private accessor to networktables
     *
     * @param id Pipeline ID
     */
    private void setPipeline(final int id) {
        pipeline.setNumber(id);
    }

    public void setPipeline(final Pipeline p) {
        this.currentPipeline = p;
        setPipeline(p.standardID);
    }

    public void setPipeline(final Pipeline p, boolean magnify) {
        this.currentPipeline = p;
        setPipeline(magnify ? p.magnifiedID : p.standardID);
    }

    public void setFloorPipeline(int index) {
        floorPipeline.setNumber(index);
    }

    public double getPipelineNumber() {
        return pipeline.getDouble(0.0);
    }

    public double getTX() {
        return combinedTarget.get(0).getDouble(0.0);
    }

    public double getTY() {
        return combinedTarget.get(1).getDouble(0.0);
    }

    public double getTA() {
        return combinedTarget.get(2).getDouble(0.0);
    }

    public double getTV() {
        return (combinedTarget.get(3).getDouble(0.0));
    }

    public double getFloorTV() {
        return (floorCombinedTarget.get(3).getDouble(0.0));
    }

    public boolean isFloorTargetVisible() {
        return getFloorTV() > 0.5;
    }

    public double[] getTranslation() {
        return ct.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
    }

    public boolean isTargetVisible() {
        if (!Robot.bot.hasMotorizedHood())
            popLimelight();

        return getTV() == 1.0;
    }

    public void magnifyCurrentPipeline() {
        setPipeline(Pipeline.POWER_PORT.magnifiedID);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculation routines
    //////////////////////////////////////////////////////////////////////////////////////////////////

    public void shrinkCurrentPipeline() {
        setPipeline(Pipeline.POWER_PORT.standardID);
    }

    /**
     * Calculates the horizontal straight-line distance from the camera to the outer
     * goal
     *
     * @return distance, in inches
     */
    public double distFromTarget() {
        double distance = (Constants.HEIGHT_OF_OUTER_PORT - Robot.bot.limelightFloorClearance)
                / Math.tan(Math.toRadians(ty + Robot.bot.limelightVerticalAngle));
        SmartDashboard.putNumber("distance from target", distance);
        return distance;
    }

    public void powerPortOuterHeadingLock() {
        refreshStandard(Pipeline.POWER_PORT);
        double heading_error = tx, steering_adjust = 0.0;

        if (Math.abs(heading_error) < Robot.bot.visionTargetingHeadingTxTolerance) {
            steering_adjust = Robot.bot.visionTargetingHeadingKp * heading_error;
        } else {
            steering_adjust = Robot.bot.visionTargetingHeadingMinPower * Math.signum(heading_error);
        }

        SmartDashboard.putBoolean("Vision heading within tolerance",
                Math.abs(steering_adjust) <= Robot.bot.visionTargetingHeadingMinPower);

        Swerve.getInstance().updateVisionLimelightAngleInput(steering_adjust);
    }

    public double getPowerPortTurretError() {
        refreshStandard(Pipeline.POWER_PORT);

        SmartDashboard.putNumber("Filtered tx", tx);

        return tx;
    }

    public double getPowerPortArea() {
        refreshStandard(Pipeline.POWER_PORT);

        return ta;
    }

    /**
     * Should snap straight first
     */
    public void powerPortOuterStrafe() {
        refreshStandard(Pipeline.POWER_PORT);
        double strafe_error = tx - 1.0, strafe_adjust = 0;

        if (strafe_error > Robot.bot.visionTargetingStrafeTxTolerance) {
            strafe_adjust = Robot.bot.visionTargetingStrafeKp * strafe_error - Robot.bot.visionTargetingStrafeMinPower;
        } else if (tx < Robot.bot.visionTargetingStrafeTxTolerance) {
            strafe_adjust = Robot.bot.visionTargetingStrafeKp * strafe_error + Robot.bot.visionTargetingStrafeMinPower;
        }

        strafe_adjust = Math.signum(distFromOuterTargetX()) * Math.abs(strafe_adjust);

        Swerve.getInstance().updateVisionTranslation((new Translation2d(strafe_adjust, 0)),
                VisionTranslationControlMode.LIMELIGHT);
    }

    public void powerPortRangeAndStrafe(double targetRange) {
        this.currentTargetRange = targetRange;
        refreshStandard(Pipeline.POWER_PORT);
        double strafe_error = tx - 1.0, strafe_adjust = 0, range_error = dist - targetRange, range_adjust = 0;

        double strafeKP = 0.03;

        if (strafe_error > Robot.bot.visionTargetingStrafeTxTolerance) {
            strafe_adjust = strafeKP * strafe_error;
        } else if (tx < Robot.bot.visionTargetingStrafeTxTolerance) {
            strafe_adjust = strafeKP * strafe_error;
        } else {
            strafe_adjust = Math.signum(strafe_error) * Robot.bot.visionTargetingStrafeMinPower;
        }

        if (range_error > Robot.bot.visionTargetingRangeTolerance) {
            range_adjust = Robot.bot.visionTargetingRangeKp * range_error;
        } else if (range_error < Robot.bot.visionTargetingRangeTolerance) {
            range_adjust = Robot.bot.visionTargetingRangeKp * range_error;
        } else {
            range_adjust = Math.signum(range_error) * Robot.bot.visionTargetingRangeMinPower;
        }

        Swerve.getInstance().updateVisionTranslation((new Translation2d(strafe_adjust, range_adjust)),
                VisionTranslationControlMode.LIMELIGHT);

    }

    public double distFromOuterTargetX() {
        return Math.sin(Math.toRadians(tx + RobotState.getInstance().getCurrentTheta())) * dist;
    }

    public double distFromOuterTargetY() {
        return Math.cos(Math.toRadians(tx + RobotState.getInstance().getCurrentTheta())) * dist;
    }

    public boolean getDistanceOnTarget() {
        return (Math.abs(currentTargetRange - dist) < Robot.bot.visionTargetingRangeTolerance);
    }

    /**
     * Calculate yaw to inner port
     *
     * @return yaw, in degrees
     */
    public double yawToInnerPort() {
        double distance = distFromTarget();
        double gyroDegrees = RobotState.getInstance().getCurrentTheta();

        double txRadians = Math.toRadians(tx);

        if (gyroDegrees == 0) {
            gyroDegrees += Util.kEpsilon;
        }

        gyroDegrees = (gyroDegrees > 180) ? gyroDegrees - 360 : gyroDegrees;

        double gyroRadiansAbsoluteValue = Math.abs(Math.toRadians(gyroDegrees));
        double phi = Math.toDegrees(
                Math.atan(((Constants.DEPTH_OF_OUTER_PORT + distance * Math.cos(txRadians + gyroRadiansAbsoluteValue))
                        / (distance * Math.sin(txRadians + gyroRadiansAbsoluteValue)))));

        double result = (phi > 0) ? (90 - phi - gyroDegrees) : -(90 + phi - gyroDegrees);

        SmartDashboard.putNumber("Gyro angle (degrees)", gyroDegrees);
        SmartDashboard.putNumber("Phi", phi);
        SmartDashboard.putNumber("Yaw to inner port (degrees)", result);

        return result;
    }

    public Translation2d vectorToOuterTarget() {
        return new Translation2d(distFromOuterTargetX(), distFromOuterTargetY());
    }

    public boolean onTarget() {
        return Math.abs(tx - 1.0) < Robot.bot.strafeTXTolerance && tx != 0.0;
    }

    void updateData(final double x, final double y, final double angle, final double[] ct) {
        updateData(new VisionData(x, y, angle, isTargetVisible(), ct));
    }

    private void updateData(final VisionData data) {
        if (collectedData.size() >= 15) {
            collectedData.removeFirst();
        }

        collectedData.add(data);
    }

    public enum Pipeline {
        DRIVER(Robot.bot.hasMotorizedHood() ? 0 : 1), POWER_PORT(1, 2), LOADING(3);

        int standardID, magnifiedID;

        Pipeline(final int standardID) {
            this(standardID, standardID);
        }

        Pipeline(final int standardID, final int magnifiedID) {
            this.standardID = standardID;
            this.magnifiedID = magnifiedID;
        }
    }

    private class VisionData {
        public final double x, y, angle;
        public final boolean seesTarget;
        public final double[] camtran;

        VisionData(final double x, final double y, final double angle, final boolean seesTarget,
                final double[] camtran) {
            this.x = x;
            this.y = y;
            this.angle = angle;
            this.seesTarget = seesTarget;
            this.camtran = camtran;
        }

        @Override
        public String toString() {
            return "x: " + x + "y: " + y + "angle: " + angle + "sees: " + seesTarget + "Trans"
                    + Arrays.toString(camtran);
        }
    }
}