package org.frogforce503.robot2021.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import org.frogforce503.lib.util.InterpolatingDouble;
import org.frogforce503.lib.util.InterpolatingTreeMap;
import org.frogforce503.robot2021.Robot;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends Subsystem {

    private static final double kEncMin = 0.0;
    private static final double kEncMax = 627;
    private static final double kAngleMin = -5.0;
    private static final double kAngleMax = 42.0;
    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToHoodAngle = new InterpolatingTreeMap<>();
    private static Hood instance = null;

    static {
        kDistanceToHoodAngle.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
        kDistanceToHoodAngle.put(new InterpolatingDouble(76.0), new InterpolatingDouble(290.0));
        kDistanceToHoodAngle.put(new InterpolatingDouble(100.0), new InterpolatingDouble(390.0));
        kDistanceToHoodAngle.put(new InterpolatingDouble(130.0), new InterpolatingDouble(500.0));
        kDistanceToHoodAngle.put(new InterpolatingDouble(160.0), new InterpolatingDouble(530.0));
        kDistanceToHoodAngle.put(new InterpolatingDouble(190.0), new InterpolatingDouble(550.0));
        kDistanceToHoodAngle.put(new InterpolatingDouble(240.0), new InterpolatingDouble(570.0));
        kDistanceToHoodAngle.put(new InterpolatingDouble(300.0), new InterpolatingDouble(590.0));

    }

    private CANCoder encoder = null;
    private Servo leftHood = null;
    private Servo rightHood = null;
    // Theta PID cos controllers;
    private final PIDController hoodPID;
    private final ArmFeedforward hoodFeedforward;
    HoodControlState controlState;
    PeriodicIO periodicIO;
    private Solenoid hoodSolenoid;

    public Hood() {
        if (Robot.bot.hasHoodSolenoid) {
            // hoodSolenoid = new Solenoid(Robot.bot.hoodID);
        } else {
            leftHood = new Servo(Robot.bot.leftHoodID);
            rightHood = new Servo(Robot.bot.rightHoodID);
            encoder = new CANCoder(Robot.bot.hoodEncoderID);

            leftHood.setBounds(2.45, 1.5, 1.5, 1.5, 0.55);
            rightHood.setBounds(2.45, 1.5, 1.5, 1.5, 0.55);

            controlState = HoodControlState.AUTO;
            periodicIO = new PeriodicIO();

            encoder.setPosition(0, 10);
            zeroSensors();
            setOpenLoop(0.0);
        }

        hoodPID = new PIDController(Robot.bot.kHoodP, Robot.bot.kHoodI, Robot.bot.kHoodD);
        hoodFeedforward = new ArmFeedforward(Robot.bot.kHoodStatic, Robot.bot.kHoodCos, 0.0, 0.0);
    }

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    /**
     * Converts a given angle in a range to its corresponding encoder reading
     *
     * @param angle Angle in range
     * @return Encoder reading in range
     */
    private static double angleToEncoder(double angle) {
        double ratio = (angle - Robot.bot.kHoodHorizontalOffset) / (kAngleMax - kAngleMin);
        return ratio * (kEncMax - kEncMin);
    }

    /**
     * Converts a given encoder reading in a range to its corresponding angle
     *
     * @param encoder Encoder reading in range
     * @return Angle in range
     */
    private static double encoderToAngle(double encoder) {
        double ratio = (encoder - kEncMin) / (kEncMax - kEncMin);
        return (ratio * (kAngleMax - kAngleMin)) - 5.0;
    }

    /**
     * Sets speed to periodic IO object. @see writePeriodicOutputs for usage
     *
     * @param speed the speed to set to the servos
     */
    private void set(double speed) {
        periodicIO.demand = speed;
    }

    /**
     * Returns encoder position of hood. @see readPeriodicInputs for sensor reding
     *
     * @return PeriodicIO's position.
     */
    public double getCurrentPosition() {
        return periodicIO.position;
    }

    public double getEncoderSetpoint() {
        return periodicIO.demand;
    }

    public void setEncoderSetpoint(double encoder) {
        controlState = HoodControlState.AUTO;

        periodicIO.demand = encoder;
    }

    /**
     * Similar to getCurrentPosition but returns in angle
     *
     * @return Angle of hood
     */
    public double getAngle() {
        return encoderToAngle(getCurrentPosition());
    }

    /**
     * Non-periodic function to drive the turret to a specific angle
     *
     * @param angle Angle to target
     */
    public void setAngle(double angle) {
        double enc = angleToEncoder(angle);
        setEncoderSetpoint(enc);
    }

    public double getHoodAngleForDistance(double distance) {
        return kDistanceToHoodAngle
                .getInterpolated(new InterpolatingDouble(Math.max(Math.min(distance, 300.0), 0.0))).value;
    }

    public void aimHoodWithVision() {
        controlState = HoodControlState.VISION;
        periodicIO.demand = 0.0;
    }

    public void aimHoodToDistance(double distance) {
        controlState = HoodControlState.DISTANCE;
        periodicIO.demand = distance;
    }

    /**
     * Set the value of speed manually
     *
     * @param speed speed to travel at
     */
    public void setOpenLoop(double speed) {
        controlState = HoodControlState.OPEN_LOOP;
        set(speed);
    }

    /**
     * Get's the current control state @see HoodControlState
     *
     * @return Current control state(auto or manual)
     */
    public HoodControlState getControlState() {
        return controlState;
    }

    /**
     * Periodic function to read from encoder and add it to periodic io object
     */
    @Override
    public synchronized void readPeriodicInputs() {
        if (!Robot.bot.hasHoodSolenoid)
            periodicIO.position = encoder.getPosition();
    }

    /**
     * Periodic function to write to the servo speeds @see profile PID calculates
     * setpoint if auto
     */
    @Override
    public synchronized void writePeriodicOutputs() {
        if (!Robot.bot.hasHoodSolenoid) {
            double output;
            if (controlState == HoodControlState.AUTO) {
                double curPos = getCurrentPosition();

                output = hoodPID.calculate(curPos, periodicIO.demand)
                        + hoodFeedforward.calculate(Math.toRadians(encoderToAngle(periodicIO.demand)), 0);

            } else if (controlState == HoodControlState.VISION) {
                double curPos = getCurrentPosition();
                double targetPos = getHoodAngleForDistance(LimelightProcessor.getInstance().distFromTarget());

                if (LimelightProcessor.getInstance().getTV() == 0) {
                    targetPos = 400;
                }

                periodicIO.demand = targetPos;

                output = hoodPID.calculate(curPos, targetPos)
                        + hoodFeedforward.calculate(encoderToAngle(periodicIO.demand), 0);
            } else if (controlState == HoodControlState.DISTANCE) {
                double curPos = getCurrentPosition();
                double targetPos = getHoodAngleForDistance(periodicIO.demand);

                output = hoodPID.calculate(curPos, targetPos)
                        + hoodFeedforward.calculate(encoderToAngle(periodicIO.demand), 0);
            } else {
                output = periodicIO.demand;
            }

            if (periodicIO.position >= kEncMax - 5.0) {
                output = Math.min(output, 0);
            }

            if (periodicIO.position <= kEncMin + 1.0) {
                output = Math.max(output, 0);
            }

            leftHood.setSpeed(output);
            rightHood.setSpeed(-output);
        }
        // hoodSolenoid.set(periodicIO.demand > (kEncMax + kEncMin) / 2);
        // }
    }

    public void zeroSensors() {
        encoder.setPosition(0, 10);
    }

    public void maxZeroSensors() {
        encoder.setPosition(627, 10);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Encoder Position", getCurrentPosition());
        SmartDashboard.putNumber("Hood Position Degrees", encoderToAngle(getCurrentPosition()));
        SmartDashboard.putNumber("Hood Demand", periodicIO.demand);
        SmartDashboard.putString("Hood State", getControlState().print());
        SmartDashboard.putNumber("Left Hood PWM", leftHood.getRaw());
        SmartDashboard.putNumber("Right Hood PWM", rightHood.getRaw());

        SmartDashboard.putData("Hood PID", hoodPID);

    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void onLoop(double timestamp) {
    }

    @Override
    public void onStop(double timestamp) {
    }

    public enum HoodControlState {
        AUTO, OPEN_LOOP, VISION, DISTANCE;

        public String print() {
            if (this == AUTO)
                return "AUTO";
            else
                return "OPEN_LOOP";
        }
    }

    public static class PeriodicIO {
        // Inputs
        public double position = 0;

        // Outputs
        public double demand;
    }
}
