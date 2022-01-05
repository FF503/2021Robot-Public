package org.frogforce503.robot2021.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode;
import org.frogforce503.robot2021.Constants;
import org.frogforce503.robot2021.Robot;

public class Spindexer extends Subsystem {

    private static final double kNeoToSpindexerRotations = 1.0 / 97.5;
    private static Spindexer instance = null;
    private final CANSparkMaxWrapper indexMotor;
    PeriodicIO periodicIO = new PeriodicIO();
    private double lastCurrentSpikeTime = 0;
    private double mixTime = 3;
    private double shootSpeed = 62.56;
    private double mixPower = 0.25;
    private SpindexerControlState state = SpindexerControlState.OFF;

    public Spindexer() {
        indexMotor = new CANSparkMaxWrapper(Robot.bot.spindexerID, MotorType.kBrushless);
        indexMotor.setInverted(true);

        indexMotor.setFF(0, 0.55 / 6100.0);
        indexMotor.setP(0, 18.000e-6);
        indexMotor.setI(0, 0.0);
        indexMotor.setD(0, 0.0);

        indexMotor.setIzone(0, 0.0);

        indexMotor.enableVoltageCompensation(12);

        setBrakeMode(false);
        setCurrentLimit(Constants.kSpindexerCurrentLim);
        stop();
    }

    public static Spindexer getInstance() {
        return instance == null ? instance = new Spindexer() : instance;
    }

    public SpindexerControlState getState() {
        return state;
    }

    public void setState(SpindexerControlState newState) {
        state = newState;
    }

    public void setBrakeMode(boolean brake) {
        indexMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void outputTelemetry() {
        if (Constants.kOutputTelemetry) {
            SmartDashboard.putNumber("Spindexer current draw", indexMotor.getOutputCurrent());
            SmartDashboard.putNumber("Spindexer Motor RPM", periodicIO.rpm);
            SmartDashboard.putNumber("Spindexer RPM", periodicIO.rpm * kNeoToSpindexerRotations);
        }
    }

    @Override
    public void stop() {
        periodicIO.setpoint = 0;
        setState(SpindexerControlState.OFF);
    }

    @Override
    public void onStart(double timestamp) {
        stop();
    }

    @Override
    public void onLoop(double timestamp) {
        outputTelemetry();
    }

    @Override
    public void onStop(double timestamp) {
        stop();
    }

    @Override
    public void writePeriodicOutputs() {
        if (getState() == SpindexerControlState.MIX) {
            if ((Timer.getFPGATimestamp() - .2) > lastCurrentSpikeTime && indexMotor.getOutputCurrent() > 8) {
                lastCurrentSpikeTime = Timer.getFPGATimestamp();
                mixPower *= -1;
            }
            if (Timer.getFPGATimestamp() > mixTime) {
                mixTime = .5 + Timer.getFPGATimestamp();
                mixPower = Math.signum(mixPower) * (.3 - Math.abs(mixPower));
                lastCurrentSpikeTime = Timer.getFPGATimestamp();
            }
            indexMotor.set(ControlMode.PercentOutput, mixPower);
        } else if (getState() == SpindexerControlState.SHOOTING) {
            if ((Timer.getFPGATimestamp() - .2) > lastCurrentSpikeTime && indexMotor.getOutputCurrent() > 9) {
                lastCurrentSpikeTime = Timer.getFPGATimestamp();
                shootSpeed *= -1;
            }

//            if ()
            indexMotor.set(ControlMode.Velocity, shootSpeed / kNeoToSpindexerRotations);
        } else {
            indexMotor.set(ControlMode.PercentOutput, periodicIO.setpoint);
        }
    }

    @Override
    public void readPeriodicInputs() {

        periodicIO.rpm = indexMotor.getEncoderVelocity();

    }

    public void setOpenLoop(double percentOutput) {
        setState(SpindexerControlState.OPEN_LOOP);
        periodicIO.setpoint = percentOutput;
    }

    public void setShootState(double shootSpeed) {
        lastCurrentSpikeTime = Timer.getFPGATimestamp();
        setState(SpindexerControlState.SHOOTING);
        this.shootSpeed = shootSpeed;
    }

    @Override
    public void setCurrentLimit(int limit) {
        indexMotor.setSmartCurrentLimit(limit);
    }

    public enum SpindexerControlState {
        VELOCITY, POSITION, OPEN_LOOP, OFF, MIX, SHOOTING
    }

    public class PeriodicIO {
        // INPUTS
        public double rpm;
        public double encoderPosition;

        // OUTPUTS
        public double setpoint;
    }
}