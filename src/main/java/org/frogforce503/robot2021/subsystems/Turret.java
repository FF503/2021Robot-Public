package org.frogforce503.robot2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import org.frogforce503.lib.drivers.TalonWrapper;
import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2021.Constants;
import org.frogforce503.robot2021.OI;
import org.frogforce503.robot2021.Robot;
import org.frogforce503.robot2021.RobotState;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends Subsystem {

    private static final double maxSoftLimit = 100;
    private static final double minSoftLimit = -193;
    private static Turret instance = null;
    TalonWrapper mTurretMotor;
    DigitalInput mCwLimit, mCcwLimit;
    PeriodicIO periodicIO = new PeriodicIO();
    private double turretEncoderZero = 4087;
    private boolean overrideSkew = true;
    private double targetAngle = 0.0;
    private double visionTarget = 0.0;
    private TurretControlState currentState = TurretControlState.PERCENT_ROTATION;
    private double kTranslationFeedforward = 0.0; // TODO
    private double kRotationFeedForward = 0.0; // TODO

    private Turret() {
        mTurretMotor = new TalonWrapper(Robot.bot.turretID);

        mCwLimit = new DigitalInput(0);
        mCcwLimit = new DigitalInput(1);

        mTurretMotor.configVoltageCompSaturation(12.0, 10);
        mTurretMotor.enableVoltageCompensation(true);
        mTurretMotor.configNominalOutputForward(0.0 / 12.0, 10);
        mTurretMotor.configContinuousCurrentLimit(80, 10);
        mTurretMotor.configPeakCurrentLimit(80, 10);
        mTurretMotor.configPeakCurrentDuration(100, 10);
        mTurretMotor.enableCurrentLimit(true);

        mTurretMotor.setInverted(false);
        mTurretMotor.setSensorPhase(false);
        setBrakeMode(false);

        configurationOne();
        mTurretMotor.configForwardSoftLimitThreshold(turretAngleToEncUnits(maxSoftLimit), 10);
        mTurretMotor.configReverseSoftLimitThreshold(turretAngleToEncUnits(minSoftLimit), 10);
        mTurretMotor.configForwardSoftLimitEnable(true, 10);
        mTurretMotor.configReverseSoftLimitEnable(true, 10);
        mTurretMotor.configOpenloopRamp(0.20, 10);
        mTurretMotor.configClosedloopRamp(0.20, 10);

        mTurretMotor.configPeakOutputForward(0.50, 10);
        mTurretMotor.configPeakOutputReverse(-0.50, 10);

        mTurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);

        int position = mTurretMotor.getSensorCollection().getPulseWidthPosition();

        mTurretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mTurretMotor.getSensorCollection().setQuadraturePosition(position, 100);

        setOpenLoop(0.0);

        // while (encUnitsToTurretAngle((int) mTurretMotor.getSelectedSensorPosition())
        // < -220) {
        // turretEncoderZero -= 4096;
        // }
        //
        // while (encUnitsToTurretAngle((int) mTurretMotor.getSelectedSensorPosition())
        // > 110) {
        // turretEncoderZero += 4096;
        // }
    }

    public static Turret getInstance() {
        if (instance == null)
            instance = new Turret();
        return instance;
    }

    private void configurationOne() {
        mTurretMotor.selectProfileSlot(0, 0);
        mTurretMotor.config_kP(0, 3.66, 10);
        mTurretMotor.config_kI(0, 0.01, 10);
        mTurretMotor.config_kD(0, 10.0, 10);
        mTurretMotor.config_kF(0, 0.00, 10);
        mTurretMotor.config_IntegralZone(0, 15, 10);
        mTurretMotor.configMotionCruiseVelocity(600, 10);
        mTurretMotor.configMotionAcceleration(1200, 10);
        // mTurretMotor.configMotionSCurveStrength(6);
    }

    public void setBrakeMode(boolean brake) {
        mTurretMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public TurretControlState getCurrentState() {
        return currentState;
    }

    public void setTurretAutoHome() {
        currentState = TurretControlState.AUTO_HOME;
    }

    public boolean getCwLimitPressed() {
        return !mCwLimit.get();
    }

    public boolean getCcwLimitPressed() {
        return !mCcwLimit.get();
    }

    public void resetEncoderZero(boolean isCcw) {
        turretEncoderZero = periodicIO.position + (isCcw ? 198.7 * 4096.0 / 360.0 : -109 * 4096.0 / 360.0);

        mTurretMotor.configForwardSoftLimitThreshold(turretAngleToEncUnits(maxSoftLimit), 10);
        mTurretMotor.configReverseSoftLimitThreshold(turretAngleToEncUnits(minSoftLimit), 10);
    }

    public boolean isOpenLoop() {
        return currentState == TurretControlState.PERCENT_ROTATION;
    }

    public void setOpenLoop(double output) {
        periodicIO.demand = output;
        currentState = TurretControlState.PERCENT_ROTATION;
    }

    public void moveToTarget() {
        currentState = TurretControlState.VISION_MOVING;
        mTurretMotor.config_kF(0, 2.56, 10);
    }

    public void lockOnTarget() {
        mTurretMotor.config_kF(0, 0.0, 10);
        currentState = TurretControlState.VISION_LOCKED;
    }

    public void trackFieldForward() {
        currentState = TurretControlState.FIELD_FORWARD;
        mTurretMotor.config_kF(0, 2.56, 10);
    }

    public double getAngle() {
        return encUnitsToTurretAngle(periodicIO.position);
    }

    public void setAngle(double angle) {
        if (isSensorConnected()) {
            configurationOne();

            mTurretMotor.configMotionCruiseVelocity(Robot.bot.kTurretMaxSpeed / 2.0, 10);
            mTurretMotor.configMotionAcceleration(300, 10);

            targetAngle = Math.min(Math.max(angle, minSoftLimit), maxSoftLimit);

            mTurretMotor.selectProfileSlot(0, 0);
            periodicIO.demand = turretAngleToEncUnits(targetAngle);
            currentState = TurretControlState.POSITION;
        } else {
            DriverStation.reportError("Turret encoder not detected!", false);
            stop();
        }
    }

    public double getFieldCentricAngle() {
        return getAngle() + RobotState.getInstance().getCurrentTheta() - 180.0;
    }

    public double getFieldCentricAngleToTarget() {
        return getFieldCentricAngle() + LimelightProcessor.getInstance().getPowerPortTurretError();
    }

    public double getCorrectiveSkewAngle() {

        double distance = LimelightProcessor.getInstance().distFromTarget();
        Rotation2d theta = Rotation2d.fromDegrees(-getFieldCentricAngleToTarget());

        return theta.getDegrees() - Math.atan2(distance * theta.getSin(), 29.375 + distance * theta.getCos());
    }

    public void setOverrideSkew(boolean override) {
        this.overrideSkew = override;
    }

    public boolean hasReachedTargetAngle() {
        return Math.abs(targetAngle - getAngle()) <= Robot.bot.kTurretAngleTolerance;
    }

    public double encUnitsToDegrees(double encUnits) {
        return encUnits / 4096.0 * 360.0;
    }

    public int degreesToEncUnits(double degrees) {
        return (int) (degrees / 360.0 * 4096.0);
    }

    public double encUnitsToTurretAngle(int encUnits) {
        return Robot.bot.kTurretStartingAngle + encUnitsToDegrees(encUnits - (int) turretEncoderZero);
    }

    public int turretAngleToEncUnits(double mTurretMotorAngle) {
        return (int) turretEncoderZero + degreesToEncUnits(mTurretMotorAngle - Robot.bot.kTurretStartingAngle);
    }

    public boolean isSensorConnected() {
        int pulseWidthPeriod = mTurretMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
        boolean connected = pulseWidthPeriod != 0;
        if (!connected)
            hasEmergency = true;
        return connected;
    }

    public void onLoop(double timestamp) {
        if (mTurretMotor.getStatorCurrent() > 80) {
            DriverStation.reportError("Turret current high", false);
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.position = (int) mTurretMotor.getSelectedSensorPosition(0);
        periodicIO.velocity = (int) mTurretMotor.getSelectedSensorVelocity(0);
        periodicIO.voltage = mTurretMotor.getMotorOutputVoltage();
        periodicIO.current = mTurretMotor.getStatorCurrent();
    }

    public boolean isTurretReady() {
        return LimelightProcessor.getInstance().isTargetVisible()
                && (LimelightProcessor.getInstance().getPowerPortTurretError() < 1.0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {

        if (currentState == TurretControlState.PERCENT_ROTATION) {
            mTurretMotor.set(ControlMode.PercentOutput, periodicIO.demand);
        } else {
            if (currentState == TurretControlState.AUTON_LOCKED) {
                // if (!overrideSkew) {
                // setVisionTarget(getCorrectiveSkewAngle() * 0.1);
                // }
                double desiredAngle = LimelightProcessor.getInstance().getPowerPortTurretError() + getAngle()
                        + (getVisionTarget() * LimelightProcessor.getInstance().getTV());
                desiredAngle = Math.min(Math.max(desiredAngle, minSoftLimit), maxSoftLimit);

                periodicIO.demand = turretAngleToEncUnits(desiredAngle);

                if (!LimelightProcessor.getInstance().isTargetVisible()) {
                    trackFieldForward();
                }

                double ff = 0;

                mTurretMotor.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, ff);

            }
            if (currentState == TurretControlState.VISION_LOCKED) {
                // if (!overrideSkew) {
                // setVisionTarget(getCorrectiveSkewAngle() * 0.1);
                // }
                double desiredAngle = LimelightProcessor.getInstance().getPowerPortTurretError() + getAngle()
                        + (getVisionTarget() * LimelightProcessor.getInstance().getTV());
                desiredAngle = Math.min(Math.max(desiredAngle, minSoftLimit), maxSoftLimit);

                periodicIO.demand = turretAngleToEncUnits(desiredAngle);

                if (!LimelightProcessor.getInstance().isTargetVisible()) {
                    trackFieldForward();
                }

                double ff = (1.35 * RobotState.getInstance().getFieldCentricSpeeds()[1] / 240);

                mTurretMotor.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, ff);

            } else if (currentState == TurretControlState.VISION_MOVING) {
                double desiredAngle = LimelightProcessor.getInstance().getPowerPortTurretError() + getAngle()
                        + (getVisionTarget() * LimelightProcessor.getInstance().getTV());
                desiredAngle = Math.min(Math.max(desiredAngle, minSoftLimit), maxSoftLimit);

                periodicIO.demand = turretAngleToEncUnits(desiredAngle);

                if (!LimelightProcessor.getInstance().isTargetVisible()) {
                    trackFieldForward();
                }

                if (Math.abs(LimelightProcessor.getInstance().getPowerPortTurretError()
                        + (getVisionTarget() * LimelightProcessor.getInstance().getTV())) < 1.0) {
                    lockOnTarget();
                }

                mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);

            } else if (currentState == TurretControlState.FIELD_FORWARD) {
                double desiredAngle = Util.boundAngleNeg180to180Degrees(
                        180.0 - Util.boundToScope(-225, 135, RobotState.getInstance().getCurrentTheta()));
                desiredAngle = Math.min(Math.max(desiredAngle, minSoftLimit), maxSoftLimit);

                periodicIO.demand = turretAngleToEncUnits(desiredAngle);

                if (LimelightProcessor.getInstance().isTargetVisible()) {
                    moveToTarget();
                }

                mTurretMotor.set(ControlMode.MotionMagic, periodicIO.demand);
            } else if (currentState == TurretControlState.AUTO_HOME) {
                mTurretMotor.configForwardSoftLimitEnable(false, 10);
                mTurretMotor.configReverseSoftLimitEnable(false, 10);
                periodicIO.demand = -.4;
                if (getCwLimitPressed() || getCcwLimitPressed()) {
                    mTurretMotor.configForwardSoftLimitEnable(true, 10);
                    mTurretMotor.configReverseSoftLimitEnable(true, 10);
                    trackFieldForward();
                }
                mTurretMotor.set(ControlMode.PercentOutput, periodicIO.demand);
            }
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Turret Angle", getAngle());
        SmartDashboard.putNumber("Turret Field Centric", getFieldCentricAngle());
        SmartDashboard.putNumber("Turret Angle To Target", getFieldCentricAngleToTarget());
        SmartDashboard.putNumber("Turret Calculated Skew", getCorrectiveSkewAngle());
        SmartDashboard.putNumber("Turret Encoder Zero", turretEncoderZero);

        if (Constants.kOutputTelemetry) {
            SmartDashboard.putNumber("Turret Current", periodicIO.current);
            SmartDashboard.putNumber("Turret Voltage", periodicIO.voltage);
            SmartDashboard.putNumber("Turret Encoder", periodicIO.position);
            SmartDashboard.putNumber("Turret Demand", periodicIO.demand);
            SmartDashboard.putString("Turret State", getCurrentState().name());
            SmartDashboard.putNumber("Turret Arb Demand", -512 * OI.getDriverRightXValue());
            SmartDashboard.putNumber("Turret Pulse Width Position",
                    mTurretMotor.getSensorCollection().getPulseWidthPosition());
            SmartDashboard.putNumber("Turret Velocity", periodicIO.velocity);
            SmartDashboard.putNumber("Turret Error", encUnitsToDegrees(mTurretMotor.getClosedLoopError(0)));
            if (mTurretMotor.getControlMode() == ControlMode.MotionMagic)
                SmartDashboard.putNumber("Turret Setpoint", mTurretMotor.getClosedLoopTarget(0));
            SmartDashboard.putBoolean("Turret Cw Limit", getCwLimitPressed());
            SmartDashboard.putBoolean("Turret Ccw Limit", getCcwLimitPressed());
        }
    }

    public double getVisionTarget() {
        return visionTarget;
    }

    public void setVisionTarget(double visionTarget) {
        this.visionTarget = visionTarget;
    }

    public boolean checkSystem() {
        double currentMinimum = 0.5;
        double currentMaximum = 20.0;

        boolean passed = true;

        if (!isSensorConnected()) {
            System.out.println("Turret sensor is not connected, connect and retest");
            return false;
        }

        double startingEncPosition = mTurretMotor.getSelectedSensorPosition(0);
        mTurretMotor.set(ControlMode.PercentOutput, 3.0 / 12.0);
        Timer.delay(1.0);
        double current = mTurretMotor.getStatorCurrent();
        mTurretMotor.set(ControlMode.PercentOutput, 0.0);
        if (Math.signum(mTurretMotor.getSelectedSensorPosition(0) - startingEncPosition) != 1.0) {
            System.out.println("Turret needs to be reversed");
            passed = false;
        }
        if (current < currentMinimum) {
            System.out.println("Turret current too low: " + current);
            passed = false;
        } else if (current > currentMaximum) {
            System.out.println("Turret current too high: " + current);
            passed = false;
        }

        return passed;
    }

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onStop(double timestamp) {

    }

    public enum TurretControlState {
        PERCENT_ROTATION, VISION_LOCKED, AUTON_LOCKED, VISION_MOVING, POSITION, FIELD_FORWARD, AUTO_HOME
    }

    public static class PeriodicIO {
        // Inputs
        public int position;
        public int velocity;
        public double voltage;
        public double current;

        // Outputs
        public double demand;
    }
}
