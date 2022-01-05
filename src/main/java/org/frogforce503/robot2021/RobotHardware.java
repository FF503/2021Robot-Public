/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import org.frogforce503.lib.util.FrogPIDF;
import org.frogforce503.lib.util.SwerveTranslationalPID;
import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2021.RobotState.Bot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Add your docs here.
 */
public abstract class RobotHardware {

    private static RobotHardware instance = null;

    // Constants
    public final String motionProfilingRioFolder = "/home/lvuser/MotionProfiles/";
    // Pure Pursuit
    public final double POSE_LOOP_DT = 0.01;
    // SwerveFileNames
    public String backLeftName;
    public String backRightName;

    // Gamespec
    public String frontLeftName;
    public String frontRightName;
    public int climberID;
    public int climberReverseID;
    public int climberForwardID;

    // Shooter
    public int shooterMasterID;
    public int shooterFollowerID;
    public double kFlywheelRevsPerMotor;
    public double kFlywheelDiameter;
    public int FEET_25_RPM; // 22 deg
    public int FEET_10_RPM; // 35 deg
    public int kFlywheelIdleVelocity;
    // Shooter PID (F, SVA)
    public double kShooterNeoKP;
    public double kShooterNeoKI;
    public double kShooterNeoKD;
    public double kShooterNeoKS;
    public double kShooterNeoKV;
    public double kShooterNeoKA;
    public double kShooterNeoIZone;
    // Spindexer
    public int spindexerID;
    public int limitID;
    public double kSpindexerTargetRpm;
    public double kSpindexerRevsPerMotor;
    public double kSpindexerIntakePos;
    public double kSpindexerPosMaxVel;
    public double kSpindexerPosMaxAccel;
    // Spindexer PIDF
    public double kSpindexerP;
    public double kSpindexerI;
    public double kSpindexerD;
    public double kSpindexerS;
    public double kSpindexerV;
    public double kSpindexerA;
    public double kSpindexerPosP;
    public double kSpindexerPosI;
    public double kSpindexerPosD;
    public double kSpindexerPosF;
    // Intake
    public int intakeID;
    public int shiftForwardID;
    public int shiftReverseID;

    // Turret
    public int turretID;
    public int kTurretMaxSpeed;
    public int kTurretAngleTolerance;
    public int kTurretStartingAngle;

    // Hood
    public int leftHoodID;
    public int rightHoodID;
    public int hoodEncoderID;
    public double kHoodHorizontalOffset = 24;
    public double kHoodP;
    public double kHoodI;
    public double kHoodD;
    public double kHoodCos;
    public double kHoodStatic;
    public boolean hasHoodSolenoid;

    public int hoodID;

    // Panel Spinner
    public int panelSpinnerID;
    public int panelSpinnerSolenoidID;
    // Limelight Constants
    public double visionAreaConstant = 1.0;
    public double yVisionkP = 0.3;
    public double xVisionkP = 0.8;
    public double limelightVerticalAngle;
    public double limelightFloorClearance;
    public double magnifyThreshold = 0.9;
    public double strafeTXTolerance = 1.5;// 0.25;
    public double limelightPopTime = 0.5;
    // Shooter Constants
    public double visionTargetingHeadingKp;
    public double visionTargetingHeadingMinPower;
    public double visionTargetingHeadingTxTolerance;
    public double visionTargetingStrafeKp;
    public double visionTargetingStrafeMinPower;
    public double visionTargetingStrafeTxTolerance;
    public double visionTargetingRangeKp;
    public double visionTargetingRangeMinPower;
    public double visionTargetingRangeTolerance;
    // Swerve Calculations Constants (measurements are in inches)
    public double kWheelbaseLength;
    public double kWheelbaseWidth;
    public double wheelDiameter;
    public double kTurnEncoderClicksperRevolution;
    public double requestDriveReversed;
    public int requestPigeonFlipped;
    // Swerve Module Positions (relative to the center of the drive base)
    public Translation2d kVehicleToFrontRight;
    public Translation2d kVehicleToBackRight;
    public Translation2d kVehicleToFrontLeft;

    // Swerve module calculation positions (weird cooridate system that we just kind
    // of have to deal with for CoR bs)

    // @formatter:off
    public Translation2d kVehicleToBackLeft;
    public Translation2d[] kModulePositions;
    /*****
     *               ^
     *               +x
     *               |
     *               |
     *               |
     *               |
     *               |
     * <+y ------------------------ -y>
     *               |
     *               |
     *               |
     *               |
     *               |
     *               -x
     *               V
     */
    // @formatter:on
    public Translation2d kFrontLeftRadius;
    public Translation2d kFrontRightRadius;
    public Translation2d kBackRightRadius;
    public Translation2d kBackLeftRadius;
    public double kMinLookAhead;
    public double kMinLookAheadSpeed;
    public double kMaxLookAhead;
    public double kMaxLookAheadSpeed;

    public double kPathFollowingMaxAccel;
    public double kPathFollowingMaxVel;

    public double kPurePursuitP;
    public double kPurePursuitV;

    public double kSwerveModuleGearRatio;

    public double kDriveNeoKP;
    public double kDriveNeoKV;
    public double kDriveNeoKS;
    public double kDriveNeoKA;

    public FrogPIDF autuonomousAnglePID;
    public FrogPIDF visionAnglePID;
    public FrogPIDF stabilizationPID;
    public FrogPIDF snappingPID;
    public SwerveTranslationalPID visionTranslationalPID;

    // public final Lookahead getLookahead() {
    // final Lookahead lookahead = new Lookahead(kMinLookAhead, kMaxLookAhead,
    // kMinLookAheadSpeed, kMaxLookAheadSpeed);
    // return lookahead;
    // }

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }

    public static RobotHardware getInstance() {
        if (instance == null) {
            if (RobotState.getInstance().getCurrentRobot().equals(Bot.Automatic)) {
                RobotState.getInstance().setCurrentRobot(Util.parseRobotNameToEnum(Util.readRobotName()));
            }
            switch (RobotState.getInstance().getCurrentRobot()) {
            case ProgrammingBot:
                instance = new RobotHardwareProgammingBot();
                break;
            case CompBot:
                instance = new RobotHardwareCompBot();
                break;
            case LegacyBot:
                instance = new RobotHardwareLegacyBot();
                break;
            case PracticeBot:
                instance = new RobotHardwarePracticeBot();
                break;
            case Automatic:
            default:
                System.err.println("Robot should not be set to automatic... something went wrong");
                break;
            }
            instance.initializeConstants();
            // Util.setPseudoInverseForwardKinematicsMatrix();
        }
        return instance;
    }

    public abstract void initializeConstants();

    public boolean hasPanelSpinner() {
        return false;
    }

    public boolean hasMotorizedHood() {
        return true;
    }

}
