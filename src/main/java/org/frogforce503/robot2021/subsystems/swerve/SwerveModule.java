package org.frogforce503.robot2021.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2021.Robot;
import org.frogforce503.robot2021.RobotState;
import org.frogforce503.robot2021.RobotState.Bot;
import org.frogforce503.robot2021.subsystems.Subsystem;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends Subsystem {
    private static final double kTurnEncoderClicksperRevolution = Robot.bot.kTurnEncoderClicksperRevolution;
    private static final double kWheelDiameter = Robot.bot.wheelDiameter;
    private static final double kAzimuthDegreesPerClick = 360.0 / kTurnEncoderClicksperRevolution;
    private static final int kSlotIdx = 0;
    private static final int kTimeoutMs = 100;
    private static final double kAzimuthClicksPerDegree = kTurnEncoderClicksperRevolution / 360.0;
    // private final int countsPerRotation = 42; // Counts/Rotation
    private final double driveGearRatio = Robot.bot.kSwerveModuleGearRatio; // Unitless
    private final double CIRCUMFERENCE = (Math.PI * kWheelDiameter); // Rotations/inch
    private final double REVS_PER_INCH = (1.0 / driveGearRatio) / CIRCUMFERENCE; // Counts/Inch
    private final double driveVelocityConversionFactor = (REVS_PER_INCH * 60.0); // 60 Inches/Rotation
    public CANSparkMaxWrapper driveMotor;
    public TalonSRX turnMotor;
    private final CANEncoder motorEncoder;

    // Swerve Module Specific - must be changed for each swerve module !!!!!
    private final String moduleName;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    private final int kBaseEncoderClicks;
    private final int kMagicCruiseVelocity;
    private final int kMagicCruiseAcceleration;
    private final boolean kTurnCountsDecreasing;
    private final boolean kDriveMotorInverted;
    private final boolean kDriveEncoderInverted;
    private final boolean kTurnMotorInverted;
    private final boolean kTurnEncoderInverted;

    private String identifier = "";

    public SwerveModule(String moduleName, int driveMotorID, int turnMotorID, double P, double I, double D, double F,
            int startingEncoderClick, int cruiseVelocity, int cruiseAccel, boolean turnCountsDecreasing,
            boolean DriveInverted, boolean DriveEncoderInverted, boolean TurnMotorInverted,
            boolean TurnEncoderInverted) {

        this.driveMotor = new CANSparkMaxWrapper(driveMotorID, MotorType.kBrushless);
        this.motorEncoder = driveMotor.getEncoder();
        this.turnMotor = new TalonSRX(turnMotorID);
        this.moduleName = moduleName;

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setNeutralMode(NeutralMode.Brake);

        // this is the encoder count when the wheel is aligned forward at the start
        this.kBaseEncoderClicks = startingEncoderClick;
        this.kMagicCruiseVelocity = cruiseVelocity;
        this.kMagicCruiseAcceleration = cruiseAccel;
        this.kTurnCountsDecreasing = turnCountsDecreasing;
        this.kDriveMotorInverted = DriveInverted;
        this.kDriveEncoderInverted = DriveEncoderInverted;
        this.kTurnMotorInverted = TurnMotorInverted;
        this.kTurnEncoderInverted = TurnEncoderInverted;
        this.kP = P;
        this.kI = I;
        this.kD = D;
        this.kF = F;

        // configure drive motor
        driveMotor.setInverted(kDriveMotorInverted);
        // set drive motor update speed - in ms

        // driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        // driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        // driveMotor.setOpenLoopRampRate(1.0);

        turnMotor.configFeedbackNotContinuous(true, kTimeoutMs);

        // set to true to invert sensor
        turnMotor.setInverted(kTurnMotorInverted);
        turnMotor.setSensorPhase(kTurnEncoderInverted);

        turnMotor.configNominalOutputForward(0, kTimeoutMs);
        turnMotor.configNominalOutputReverse(0, kTimeoutMs);
        turnMotor.configPeakOutputForward(1, kTimeoutMs);
        turnMotor.configPeakOutputReverse(-1, kTimeoutMs);
        turnMotor.selectProfileSlot(kSlotIdx, 0); // slot index = 0 , pidloopidx = 0
        turnMotor.config_kF(kSlotIdx, kF, kTimeoutMs);
        turnMotor.config_kP(kSlotIdx, kP, kTimeoutMs);
        turnMotor.config_kI(kSlotIdx, kI, kTimeoutMs);
        turnMotor.config_kD(kSlotIdx, kD, kTimeoutMs);
        turnMotor.configMotionCruiseVelocity(kMagicCruiseVelocity, kTimeoutMs);
        turnMotor.configMotionAcceleration(kMagicCruiseAcceleration, kTimeoutMs);

        driveMotor.setP(0, Robot.bot.kDriveNeoKP);
        // driveMotor.setI(0, 0.0);
        // driveMotor.setD(0, 0);
        driveMotor.setFF(0, Robot.bot.kDriveNeoKV);

        driveMotor.setSmartCurrentLimit(50);
        turnMotor.enableCurrentLimit(true);
        turnMotor.configContinuousCurrentLimit(15);
        // configure turn motor
        switch (RobotState.getInstance().getCurrentRobot()) {
        case CompBot:
        case LegacyBot:
        case PracticeBot:
            // turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
            // kSlotIdx, kTimeoutMs);
            turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

            int position = turnMotor.getSensorCollection().getPulseWidthPosition();
            // if (position<0){
            // position += 4096;
            // }
            // System.out.println(position);
            turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kSlotIdx, 100);
            turnMotor.getSensorCollection().setQuadraturePosition(position, 100);
            // turnMotor.getSensorCollection().setPulseWidthPosition(position, 100);
            // turnMotor.getSensorCollection().setAnalogPosition(position,100);
            // turnMotor.setSelectedSensorPosition(position);
            break;
        case ProgrammingBot:
        default:
            turnMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
            break;
        }
        // motorEncoder.setVelocityConversionFactor(driveVelocityConversionFactor);
    }

    public String getID() {
        return identifier;
    }

    public void setID(String id) {
        identifier = id;
    }

    public void drive(SwerveModuleState desiredState) {
        drive(desiredState.speedMetersPerSecond, desiredState.angle.getDegrees());
    }

    public void drive(double speed, double angle) {
        // if (this.moduleName.equals("PracticeFrontLeft"))
        // System.out.println(this.moduleName + ": angle: " + angle);
        double trueSpeed = speed;
        if (speed == 503.0) {
            trueSpeed = 0.0;
        }
        setDriveMotorSpeed(trueSpeed);

        // angle is bound to -180 - +180 and degrees are from 0-360
        // convert bounded angle into true compass degrees
        // double trueAngle = angle;
        // if (angle < 0) {
        // trueAngle = 180 + (180 + angle);
        // }
        double trueAngle = Util.boundAngle0to360Degrees(angle);

        // convert angle (0-360) into encoder clicks (0-1024)
        int desiredclicks = (int) Math.round(trueAngle * kAzimuthClicksPerDegree);

        if (kTurnCountsDecreasing) {
            // this means a positive right turn mean decreasing encoder counts
            desiredclicks = kBaseEncoderClicks - desiredclicks;
            if (desiredclicks < 0) {
                desiredclicks += kTurnEncoderClicksperRevolution;
            }

        } else {
            // this means a positive right trn with increasing encoder counts
            // addin the base starting clicks when the wheel is pointing to zero
            desiredclicks += kBaseEncoderClicks;
            // becuase we are using an absolute encoder the value must be between 0 and 1024
            if (desiredclicks >= kTurnEncoderClicksperRevolution) {
                desiredclicks -= kTurnEncoderClicksperRevolution;
            }
        }
        double pos = turnMotor.getSelectedSensorPosition(); // sensor units
        int rotationNumber = (int) (pos / kTurnEncoderClicksperRevolution);
        if (pos < 0) {
            rotationNumber--;
        }
        int target;
        // Only make rotation correction on mag encoder swerve bots
        if (!RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
            desiredclicks += rotationNumber * kTurnEncoderClicksperRevolution;
            // System.out.println("in cur rot: " + desiredclicks);

            int desiredclicksPrevRotation = desiredclicks - (int) kTurnEncoderClicksperRevolution;
            int desiredClicksNextRotation = desiredclicks + (int) kTurnEncoderClicksperRevolution;
            // System.out.println("in next rot: " + desiredClicksNextRotation);
            // System.out.println("in prev rot: " + desiredclicksPrevRotation);
            target = getClosestPointToTarget((int) pos, desiredclicks, desiredClicksNextRotation,
                    desiredclicksPrevRotation);
        } else {
            target = desiredclicks;
        }

        if (Math.abs(speed) > 0.02 || speed == 503.0) {
            turnMotor.set(ControlMode.MotionMagic, target);
        }
    }

    public static double encoderToAngle(int encoder) {
        return encoder / kAzimuthClicksPerDegree;
    }

    public int getBaseEncoderClicks() {
        return kBaseEncoderClicks;
    }

    public void driveWithVelocity(double velocity, double angle) {
        // if (this.moduleName.equals("PracticeFrontLeft"))
        // System.out.println(this.moduleName + ": angle: " + angle);

        driveMotor.set(org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode.Velocity,
                velocity * REVS_PER_INCH * 60, Math.signum(velocity) * Robot.bot.kDriveNeoKS);

        // System.out.println("Closed loop error(RPM): " +
        // driveMotor.getClosedLoopError());
        // setDriveMotorSpeed(velocity);

        // angle is bound to -180 - +180 and degrees are from 0-360
        // convert bounded angle into true compass degrees
        // double trueAngle = angle;
        // if (angle < 0) {
        // trueAngle = 180 + (180 + angle);
        // }
        double trueAngle = Util.boundAngle0to360Degrees(angle);

        // convert angle (0-360) into encoder clicks (0-1024)
        int desiredclicks = (int) Math.round(trueAngle * kAzimuthClicksPerDegree);

        if (kTurnCountsDecreasing) {
            // this means a positive right turn mean decreasing encoder counts
            desiredclicks = kBaseEncoderClicks - desiredclicks;
            if (desiredclicks < 0) {
                desiredclicks += kTurnEncoderClicksperRevolution;
            }

        } else {
            // this means a positive right trn with increasing encoder counts
            // addin the base starting clicks when the wheel is pointing to zero
            desiredclicks += kBaseEncoderClicks;
            // becuase we are using an absolute encoder the value must be between 0 and 1024
            if (desiredclicks >= kTurnEncoderClicksperRevolution) {
                desiredclicks -= kTurnEncoderClicksperRevolution;
            }
        }
        double pos = turnMotor.getSelectedSensorPosition(); // sensor units
        int rotationNumber = (int) (pos / kTurnEncoderClicksperRevolution);
        if (pos < 0) {
            rotationNumber--;
        }
        int target;
        // Only make rotation correction on mag encoder swerve bots
        if (!RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
            desiredclicks += rotationNumber * kTurnEncoderClicksperRevolution;
            // System.out.println("in cur rot: " + desiredclicks);

            int desiredclicksPrevRotation = desiredclicks - (int) kTurnEncoderClicksperRevolution;
            int desiredClicksNextRotation = desiredclicks + (int) kTurnEncoderClicksperRevolution;
            // System.out.println("in next rot: " + desiredClicksNextRotation);
            // System.out.println("in prev rot: " + desiredclicksPrevRotation);
            target = getClosestPointToTarget((int) pos, desiredclicks, desiredClicksNextRotation,
                    desiredclicksPrevRotation);
        } else {
            target = desiredclicks;
        }

        if (Math.abs(velocity) > 0.02) {
            turnMotor.set(ControlMode.MotionMagic, target);
        }
    }

    private int absoluteDistanceToTarget(int target, int pos) {
        return Math.abs(target - pos);
    }

    private int getClosestPointToTarget(int pos, int t1, int t2, int t3) {
        int d1 = absoluteDistanceToTarget(t1, pos), d2 = absoluteDistanceToTarget(t2, pos),
                d3 = absoluteDistanceToTarget(t3, pos);
        int minD = Math.min(d1, Math.min(d2, d3));
        if (d1 == minD) {
            return t1;
        } else if (d2 == minD) {
            return t2;
        }
        return t3;
    }

    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(speed);
    }

    public void resetDriveEncoder() {
        motorEncoder.setPosition(0.0);
    }

    /**
     * @return clicks
     */
    public double getDriveEncoderClicks() {
        double pos = motorEncoder.getPosition();
        if (kDriveEncoderInverted) {
            pos *= -1;
        }
        return pos;
    }

    public double getDriveEncoderRPM() {
        double vol = motorEncoder.getVelocity();
        if (kDriveEncoderInverted) {
            vol *= -1;
        }
        return vol;
    }

    public double getDriveMotorVoltage() {
        return driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    }

    /**
     * @return in inches
     */
    public double getDriveMotorPosition() {
        return getDriveEncoderClicks() / REVS_PER_INCH;
    }

    /**
     * @return in inches/second
     */
    public double getDriveMotorVelocity() {
        return getDriveEncoderRPM() / driveVelocityConversionFactor;
    }

    public SwerveModuleState getState() {
        SwerveModuleState s = new SwerveModuleState(getDriveMotorVelocity(),
                Rotation2d.fromDegrees(-getTurnEncoderPositioninDegrees()));
        return s;
    }

    public double getTurnEncoderPosition() {
        return turnMotor.getSelectedSensorPosition(0);
    }

    public double getTurnClosedLoopError() {
        return turnMotor.getClosedLoopError();
    }

    public void setDriveMotorCurrentLimit(int limit) {
        driveMotor.setSmartCurrentLimit(limit);
    }

    public void coastDrive() {
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

    public void brakeDrive() {
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    public void coastAzimuth() {
        turnMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void brakeAzimuth() {
        turnMotor.setNeutralMode(NeutralMode.Brake);
    }

    public double getTurnEncoderPositioninDegrees() {
        // get the base clicks = zero degrees for this particular wheel

        double pos = getTurnEncoderPosition();
        // relative position
        double relpos = 0;
        // actual position
        int actpos;

        if (kTurnCountsDecreasing) {
            // this means a positive right turn means decreasing encoder counts
            relpos = (kBaseEncoderClicks - pos);
        } else {
            // this means a positive right turn with increasing encoder counts
            relpos = (pos - kBaseEncoderClicks);
        }

        // adjust for absolute encoder with 0-1024 clicks
        if (relpos < 0) {
            relpos += kTurnEncoderClicksperRevolution;
        }
        actpos = (int) Math.round(relpos * kAzimuthDegreesPerClick);

        if (actpos == 360) {
            actpos = 0;
        }

        return actpos;
    }

    public String getName() {
        return this.moduleName;
    }

    /********************************************************************************
     * Section - Encoder Conversion Routines
     *******************************************************************************/

    // private static double ticksToInches(double ticks) {
    // return rotationsToInches(ticksToRotations(ticks));
    // }

    // private static double rotationsToInches(double rotations) {
    // return rotations * (kWheelDiameter * Math.PI);
    // }

    // private static double ticksToRotations(double ticks) {
    // int kEncoderUnitsPerRev = 42; // Rev Native Internal Encoder clicks per
    // revolution
    // return ticks / kEncoderUnitsPerRev;
    // }

    // private static double inchesToRotations(double inches) {
    // return inches / (kWheelDiameter * Math.PI);
    // }

    // public void printError(){
    // Log.add(this.moduleName+" azimuth ", value);
    // }
    public double getXComponentVelocity() {
        return Math.cos(Math.toRadians(Util.toUnitCircle(getTurnEncoderPositioninDegrees())))
                * driveMotor.getEncoder().getVelocity();
    }

    public double getYComponentVelocity() {
        return Math.sin(Math.toRadians(Util.toUnitCircle(getTurnEncoderPositioninDegrees())))
                * driveMotor.getEncoder().getVelocity();
    }

    public double getVelocityError() {
        return driveMotor.getClosedLoopError();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(this.moduleName + " Drive Clicks", this.getDriveEncoderClicks());
        SmartDashboard.putNumber(this.moduleName + " Drive Inches", this.getDriveMotorPosition());
        SmartDashboard.putNumber(this.moduleName + " Drive Velocity", this.getDriveMotorVelocity());
        SmartDashboard.putNumber(this.moduleName + " Turn Clicks", this.getTurnEncoderPosition());
        SmartDashboard.putNumber(this.moduleName + " Turn Degrees", this.getTurnEncoderPositioninDegrees());
        SmartDashboard.putNumber(this.moduleName + " Turn Error (clicks)", this.getTurnClosedLoopError());
    }

    @Override
    public void stop() {
        this.setDriveMotorSpeed(0.0);
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
}