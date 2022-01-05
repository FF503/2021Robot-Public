/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode;
import org.frogforce503.lib.util.InterpolatingDouble;
import org.frogforce503.lib.util.InterpolatingTreeMap;
import org.frogforce503.robot2021.Constants;
import org.frogforce503.robot2021.Robot;
import org.frogforce503.robot2021.RobotState;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToShooterSpeed = new InterpolatingTreeMap<>();
    private static Shooter instance = null;

    static {
        kDistanceToShooterSpeed.put(new InterpolatingDouble(0.0), new InterpolatingDouble(1000.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(76.0), new InterpolatingDouble(2400.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(100.0), new InterpolatingDouble(2800.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(130.0), new InterpolatingDouble(3500.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(160.0), new InterpolatingDouble(3700.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(190.0), new InterpolatingDouble(3800.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(240.0), new InterpolatingDouble(3900.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(300.0), new InterpolatingDouble(4000.0));
    }

    // PID (F, SVA) Constants
    private final double kShooterNeoKP = Robot.bot.kShooterNeoKP;
    private final double kShooterNeoKS = Robot.bot.kShooterNeoKS;
    private final double kShooterNeoKV = Robot.bot.kShooterNeoKV;
    private final double kTranslationF = 0.1;
    private final double kFlywheelRevsPerMotor = Robot.bot.kFlywheelRevsPerMotor;
    private final PeriodicIO periodicIO = new PeriodicIO();
    ControlState state = ControlState.OPEN_LOOP;
    LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    CANSparkMaxWrapper mShooter, mFollower;
    private double mTargetRpm = 0;
    private boolean velocityStabilized = true;

    public Shooter() {
        mShooter = new CANSparkMaxWrapper(Robot.bot.shooterMasterID, MotorType.kBrushless);
        mShooter.set(ControlMode.PercentOutput, 0.0);
        mShooter.setInverted(true);
        mShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mShooter.enableVoltageCompensation(12.0);

        mFollower = new CANSparkMaxWrapper(Robot.bot.shooterFollowerID, MotorType.kBrushless);
        mFollower.setInverted(true);
        mFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mFollower.enableVoltageCompensation(12.0);

        mFollower.follow(mShooter, true);

        // mSpinUpController.calculate(0, 0);
        // mFeedforward.calculate(0);

        setCurrentLimit(Constants.kShooterCurrentLimit);

        setState(ControlState.OPEN_LOOP);

        mShooter.setP(0, kShooterNeoKP);
        mShooter.setFF(0, kShooterNeoKV);
        mShooter.setIzone(0, 0);

        // mSpinUpController.reset();
        // mStabilizedController.reset();
    }

    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    public ControlState getState() {
        return state;
    }

    public void setState(ControlState newState) {
        state = newState;
    }

    private double getLastSetRpm() {
        return mTargetRpm;
    }

    @Override
    public void outputTelemetry() {
        if (Constants.kOutputTelemetry) {
            double voltage = mShooter.getBusVoltage() * mShooter.getAppliedOutput();
            SmartDashboard.putNumber("Shooter Wheel Speed", getShooterSpeedRpm());
            SmartDashboard.putNumber("Shooter Wheel Speed Error", getShooterSpeedError());
            SmartDashboard.putNumber("Shooter Motor Speed", getEncoderSpeedRpm());
            SmartDashboard.putNumber("Shooter 1 Voltage", mShooter.getBusVoltage() * mShooter.getAppliedOutput());
            SmartDashboard.putNumber("Shooter 2 Voltage", mFollower.getBusVoltage() * mFollower.getAppliedOutput());

            SmartDashboard.putNumber("Shooter Wheel Setpoint",
                    periodicIO.shooterDemand * Robot.bot.kFlywheelRevsPerMotor);
            SmartDashboard.putNumber("Shooter Motor Setpoint", periodicIO.shooterDemand);
            SmartDashboard.putNumber("Shooter Current", mShooter.getOutputCurrent());
            SmartDashboard.putNumber("Shooter Current Filtered", currentFilter.calculate(mShooter.getOutputCurrent()));
            SmartDashboard.putString("Shooter State", getState().toString());

            SmartDashboard.putNumber("Shooter Voltage", voltage);
            // SmartDashboard.put

            // Shot ready displays
            SmartDashboard.putBoolean("Shooter Ready", Math.abs(getShooterSpeedRpm() - mTargetRpm) < 100);
            SmartDashboard.putBoolean("Vision Ready", LimelightProcessor.getInstance().onTarget());
        }
    }

    @Override
    public void stop() {
        setState(ControlState.OPEN_LOOP);
        periodicIO.shooterDemand = 0.0;
        velocityStabilized = true;
    }

    @Override
    public void onStart(double timestamp) {
        setState(ControlState.OPEN_LOOP);
    }

    @Override
    public void onLoop(double timestamp) {

    }

    @Override
    public void onStop(double timestamp) {
    }

    /**
     * Run the shooter in open loop/raw voltage input
     */
    public void setOpenLoop(double shooterPercent) {
        setState(ControlState.OPEN_LOOP);
        periodicIO.shooterDemand = shooterPercent;
    }

    /**
     * Update the velocity control setpoint of the Shooter. This is the main method
     * to call for velocity control Request
     */
    public void setVelocity(double rpm) {
        setState(ControlState.VELOCITY);
        mTargetRpm = rpm;
        periodicIO.shooterDemand = (rpm / kFlywheelRevsPerMotor) * RobotState.getInstance().getShooterSpeedIncrement();
    }

    public void aimShooterSpeedWithVision() {
        // mSpinUpController.reset();
        // mStabilizedController.reset();
        setState(ControlState.VISION);
        periodicIO.shooterDemand = 0.0;
        // spinUpStage = true;
    }

    public void aimShooterSpeedToDistance(double distance) {
        setState(ControlState.DISTANCE);
        // periodicIO.shooterDemand = distance;
        // mSpinUpController.reset();
        // mStabilizedController.reset();
        // spinUpStage = true;

    }

    public double getShooterSpeedForDistance(double distance) {
        return kDistanceToShooterSpeed
                .getInterpolated(new InterpolatingDouble(Math.max(Math.min(distance, 300.0), 0.0))).value;
    }

    public boolean isShooterReady() {
        return this.velocityStabilized;
    }

    public void idle() {
        setState(ControlState.IDLE);
        mTargetRpm = Robot.bot.kFlywheelIdleVelocity;
        periodicIO.shooterDemand = Robot.bot.kFlywheelIdleVelocity / kFlywheelRevsPerMotor;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.rpm = getShooterSpeedRpm();

        periodicIO.dt = Timer.getFPGATimestamp() - periodicIO.lastTime;
        // if (periodicIO.dt > 0.1) {
        periodicIO.acceleration = (periodicIO.rpm - periodicIO.lastRpm) / periodicIO.dt;
        periodicIO.lastRpm = periodicIO.rpm;
        periodicIO.lastTime = Timer.getFPGATimestamp();
        // }

    }

    @Override
    public void writePeriodicOutputs() {
        if (getState() == ControlState.OPEN_LOOP) {
            mShooter.set(ControlMode.PercentOutput, periodicIO.shooterDemand);
        } else if (getState() == ControlState.VISION) {
            double targetVelocity = getShooterSpeedForDistance(LimelightProcessor.getInstance().distFromTarget())
                    / Robot.bot.kFlywheelRevsPerMotor;

            if (LimelightProcessor.getInstance().getTV() == 0.0) {
                targetVelocity = 3000.0 / Robot.bot.kFlywheelRevsPerMotor;
            }

            periodicIO.shooterDemand = targetVelocity;

            velocityStabilized = targetVelocity - getEncoderSpeedRpm() < 50;

            mShooter.set(ControlMode.Velocity, targetVelocity, kShooterNeoKS);
        } else if (getState() == ControlState.DISTANCE) {
            double targetVelocity = getShooterSpeedForDistance(167.0) / Robot.bot.kFlywheelRevsPerMotor;

            periodicIO.shooterDemand = targetVelocity;

            mShooter.set(ControlMode.Velocity, targetVelocity, kShooterNeoKS);
        } else {
            mShooter.set(ControlMode.Velocity, periodicIO.shooterDemand, kShooterNeoKS);
        }
    }

    public double getSetpointRpm() {
        return periodicIO.shooterDemand * kFlywheelRevsPerMotor;
    }

    private double getEncoderSpeedRpm() {
        return mShooter.getEncoderVelocity();
    }

    private double getShooterSpeedRpm() {
        return getEncoderSpeedRpm() * kFlywheelRevsPerMotor;
    }

    private double getShooterSpeedError() {
        return periodicIO.rpm - getLastSetRpm();

    }

    @Override
    public void setCurrentLimit(int limit) {
        mShooter.setSmartCurrentLimit(limit);
        mFollower.setSmartCurrentLimit(limit);
    }

    public void testShooter(double power, boolean input1, boolean input2) {
        mShooter.set(ControlMode.PercentOutput, power);
        mFollower.set(ControlMode.PercentOutput, power);
    }

    public enum ControlState {
        IDLE, OPEN_LOOP, VELOCITY, VISION, DISTANCE
    }

    public class PeriodicIO {
        // Inputs
        public double rpm;
        public double lastRpm;

        // Acceleration
        public double dt;
        public double acceleration; // Units of rot / (min * sec)
        public double lastTime = 0;

        // Outputs
        public double shooterDemand;
        public double kickerDemand;
    }

}
