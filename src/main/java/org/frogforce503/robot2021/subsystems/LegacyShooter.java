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
import org.frogforce503.robot2021.RobotState.GameState;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LegacyShooter extends Subsystem {
    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToShooterSpeed = new InterpolatingTreeMap<>();

    static {
        kDistanceToShooterSpeed.put(new InterpolatingDouble(0.0), new InterpolatingDouble(4100.0));
        // kDistanceToShooterSpeed.put(new InterpolatingDouble(76.0), new
        // InterpolatingDouble(4315.0));
        // kDistanceToShooterSpeed.put(new InterpolatingDouble(100.0), new
        // InterpolatingDouble(4383.0));
        // kDistanceToShooterSpeed.put(new InterpolatingDouble(130.0), new
        // InterpolatingDouble(4468.0));
        // kDistanceToShooterSpeed.put(new InterpolatingDouble(160.0), new
        // InterpolatingDouble(4553.0));
        // kDistanceToShooterSpeed.put(new InterpolatingDouble(190.0), new
        // InterpolatingDouble(3800.0));
        // kDistanceToShooterSpeed.put(new InterpolatingDouble(240.0), new
        // InterpolatingDouble(3900.0));
        kDistanceToShooterSpeed.put(new InterpolatingDouble(160.0), new InterpolatingDouble(6580.0));
    }

    private final double maxDist = 300;

    private static LegacyShooter instance = null;

    public static LegacyShooter getInstance() {
        if (instance == null)
            instance = new LegacyShooter();
        return instance;
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
        public double demand;
    }

    ControlState state = ControlState.OPEN_LOOP;

    public ControlState getState() {
        return state;
    }

    public void setState(ControlState newState) {
        state = newState;
    }

    CANSparkMaxWrapper mShooter, mSlave;

    Solenoid hoodSolenoid;

    // PID (F, SVA) Constants
    private final double kShooterNeoKP = Robot.bot.kShooterNeoKP;
    private final double kShooterNeoKS = Robot.bot.kShooterNeoKS;
    private final double kShooterNeoKV = Robot.bot.kShooterNeoKV;
    private final double kTranslationF = 0.1;
    private final double kFlywheelRevsPerMotor = Robot.bot.kFlywheelRevsPerMotor;
    LinearFilter currentFilter = LinearFilter.highPass(0.1, 0.02);
    CANSparkMaxWrapper mFollower;
    private boolean velocityStabilized = true;

    public enum ControlState {
        IDLE, OPEN_LOOP, VELOCITY, VISION, DISTANCE
    }

    private PeriodicIO periodicIO = new PeriodicIO();

    private double mTargetRpm = 0;
    private boolean mHoodState = false;

    private boolean isAtShootingSpeed;

    private double getLastSetRpm() {
        return mTargetRpm;
    }

    public LegacyShooter() {
        mShooter = new CANSparkMaxWrapper(Robot.bot.shooterMasterID, MotorType.kBrushless);
        mShooter.set(ControlMode.PercentOutput, 0.0);
        mShooter.setInverted(true);
        mShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mShooter.enableVoltageCompensation(12.0);

        mSlave = new CANSparkMaxWrapper(Robot.bot.shooterFollowerID, MotorType.kBrushless);
        mSlave.setInverted(true);
        mSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mSlave.enableVoltageCompensation(12.0);

        mSlave.follow(mShooter, true);

        hoodSolenoid = new Solenoid(Robot.bot.hoodID);

        // refreshControllerConsts();
        // mPidController.calculate(0, 0);
        // mFeedforward.calculate(0);
        setCurrentLimit(Constants.kShooterCurrentLimit);
        // mShooter.selectProfileSlot(0);

        setState(ControlState.OPEN_LOOP);
        mShooter.setP(0, kShooterNeoKP);
        mShooter.setFF(0, kShooterNeoKV);
        mShooter.setIzone(0, 0);
    }

    @Override
    public void outputTelemetry() {
        if (Constants.kOutputTelemetry) {
            double voltage = mShooter.getBusVoltage() * mShooter.getAppliedOutput();
            // SmartDashboard.putNumber("shooter_speed_neo", getEncoderSpeedRpm());
            SmartDashboard.putNumber("shooter_speed_flywheel", getShooterSpeedRpm());
            if (Math.abs((periodicIO.demand * kFlywheelRevsPerMotor) - getShooterSpeedRpm()) < 200) {
                SmartDashboard.putNumber("shooter_speed_error",
                        (periodicIO.demand * kFlywheelRevsPerMotor) - getShooterSpeedRpm());
            }
            SmartDashboard.putNumber("shooter1_output_voltage", mShooter.getBusVoltage() * mShooter.getAppliedOutput());
            SmartDashboard.putNumber("shooter2_output_voltage", mSlave.getBusVoltage() * mSlave.getAppliedOutput());
            // SmartDashboard.putNumber("shooter1_temp", mShooter.getMotorTemperature());
            // SmartDashboard.putNumber("shooter2_temp", mSlave.getMotorTemperature());
            SmartDashboard.putNumber("shooter_setpoint", periodicIO.demand * Robot.bot.kFlywheelRevsPerMotor);
            SmartDashboard.putNumber("shoote_current", mShooter.getOutputCurrent());
            SmartDashboard.putString("shooter state", getState().toString());

            SmartDashboard.putNumber("Shooter Voltage", voltage);
            SmartDashboard.putBoolean("Hood State", !hoodSolenoid.get());

            // Shot ready displays
            // SmartDashboard.putBoolean("Shooter Ready", isAtShootingSpeed);
            SmartDashboard.putBoolean("Vision Ready", LimelightProcessor.getInstance().onTarget());
        }
    }

    @Override
    public void stop() {
        setState(ControlState.OPEN_LOOP);
        LimelightProcessor.getInstance().dropLimelight();
        setHood(false);
        periodicIO.demand = 0.0;
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
    public void setOpenLoop(double percentOutput) {
        setState(ControlState.OPEN_LOOP);
        LimelightProcessor.getInstance().dropLimelight();
        periodicIO.demand = percentOutput;
    }

    /**
     * Update the velocity control setpoint of the Shooter. This is the main method
     * to call for velocity control Request
     */
    private void setVelocity(double rpm) {
        setState(ControlState.VELOCITY);
        LimelightProcessor.getInstance().dropLimelight();
        mTargetRpm = rpm;
        periodicIO.demand = (rpm / kFlywheelRevsPerMotor) * RobotState.getInstance().getShooterSpeedIncrement();
    }

    // private void idle() {
    // setState(ControlState.IDLE);
    // mTargetRpm = Robot.bot.kFlywheelIdleVelocity;
    // periodicIO.demand = Robot.bot.kFlywheelIdleVelocity / kFlywheelRevsPerMotor;
    // }

    public void aimShooterSpeedWithVision() {
        // mSpinUpController.reset();
        // mStabilizedController.reset();
        LimelightProcessor.getInstance().popLimelight();
        setState(ControlState.VISION);
        periodicIO.demand = 0.0;
        // spinUpStage = true;
    }

    public void aimShooterSpeedToDistance(double distance) {
        setState(ControlState.DISTANCE);
        LimelightProcessor.getInstance().dropLimelight();
        // periodicIO.shooterDemand = distance;
        // mSpinUpController.reset();
        // mStabilizedController.reset();
        // spinUpStage = true;

    }

    public double getShooterSpeedForDistance(double distance) {
        return kDistanceToShooterSpeed
                .getInterpolated(new InterpolatingDouble(Math.max(Math.min(distance, 160.0), 0.0))).value;
    }

    public boolean isShooterReady() {
        return this.velocityStabilized;
    }

    public void idle() {
        setState(ControlState.IDLE);
        mTargetRpm = Robot.bot.kFlywheelIdleVelocity;
        periodicIO.demand = Robot.bot.kFlywheelIdleVelocity / kFlywheelRevsPerMotor;
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
            mShooter.set(ControlMode.PercentOutput, periodicIO.demand);
        } else if (getState() == ControlState.VISION) {
            LimelightProcessor.getInstance().refreshStandard(LimelightProcessor.Pipeline.DRIVER);

            double distance = LimelightProcessor.getInstance().distFromTarget();

            setHood(distance < maxDist / 2);

            double targetVelocity = getShooterSpeedForDistance(distance) / Robot.bot.kFlywheelRevsPerMotor;

            if (LimelightProcessor.getInstance().getTV() == 0.0) {
                targetVelocity = 3000.0 / Robot.bot.kFlywheelRevsPerMotor;
            }

            periodicIO.demand = targetVelocity;

            velocityStabilized = targetVelocity - getEncoderSpeedRpm() < 50;

            mShooter.set(ControlMode.Velocity, targetVelocity, kShooterNeoKS);
        } else if (getState() == ControlState.DISTANCE) {
            double targetVelocity = getShooterSpeedForDistance(167.0) / Robot.bot.kFlywheelRevsPerMotor;

            periodicIO.demand = targetVelocity;

            mShooter.set(ControlMode.Velocity, targetVelocity, kShooterNeoKS);
        } else {
            mShooter.set(ControlMode.Velocity, periodicIO.demand, kShooterNeoKS);
        }
        SmartDashboard.putString("LEGACYSHOOTERSTATE", getState().name());
    }

    public double getSetpointRpm() {
        return periodicIO.demand * kFlywheelRevsPerMotor;
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
        mSlave.setSmartCurrentLimit(limit);
    }

    public void setHood(boolean input) {
        hoodSolenoid.set(input);
        this.mHoodState = input;
    }

    public boolean getHood() {
        return this.mHoodState;
    }

    public void toggleHoodState() {
        hoodSolenoid.set(!hoodSolenoid.get());
    }

    public void velocityCommand(double rpm, boolean longShot) {
        isAtShootingSpeed = false;
        System.out.println("Shooter velocity of " + rpm + " requested");
        setHood(!longShot);
        setVelocity(rpm);
    }

    public boolean isReadyToShoot() {
        boolean autonShooterReady = true;

        if (RobotState.getInstance().getGameState() == GameState.AUTON) {
            autonShooterReady = RobotState.getInstance().isAutonReadyToShoot();
        }
        boolean finished = (autonShooterReady && getState() == ControlState.VELOCITY
                && (Math.abs(getShooterSpeedError()) < 100 && mShooter.getOutputCurrent() < 18));
        if (finished) {
            isAtShootingSpeed = true;
        }
        return finished;
    }

    public void testShooter(double power, boolean input1, boolean input2) {
        mShooter.set(ControlMode.PercentOutput, power);
        mSlave.set(ControlMode.PercentOutput, power);
    }

}
