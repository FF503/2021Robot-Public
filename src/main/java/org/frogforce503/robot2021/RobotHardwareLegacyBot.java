/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021;

import org.frogforce503.lib.util.FrogPIDF;
import org.frogforce503.lib.util.FrogPIDF.ControlMode;
import org.frogforce503.lib.util.SwerveTranslationalPID;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class RobotHardwareLegacyBot extends RobotHardware {
    /* All distance measurements are in inches, unless otherwise noted */

    @Override
    public void initializeConstants() {
        // Swerve Module JSON file names
        backLeftName = "LegacyBackLeft";
        backRightName = "LegacyBackRight";
        frontLeftName = "LegacyFrontLeft";
        frontRightName = "LegacyFrontRight";

        // Swerve Calculations Constants (measurements are in inches)
        kWheelbaseLength = 20.5;
        kWheelbaseWidth = 20.5;
        wheelDiameter = 4.0;
        kTurnEncoderClicksperRevolution = 4096;
        requestDriveReversed = 1;

        requestPigeonFlipped = -1;

        // Swerve Module Positions (relative to the center of the drive base)
        kVehicleToFrontLeft = new Translation2d(kWheelbaseLength / 2, kWheelbaseWidth / 2);
        kVehicleToFrontRight = new Translation2d(kWheelbaseLength / 2, -kWheelbaseWidth / 2);
        kVehicleToBackRight = new Translation2d(-kWheelbaseLength / 2, -kWheelbaseWidth / 2);
        kVehicleToBackLeft = new Translation2d(-kWheelbaseLength / 2, kWheelbaseWidth / 2);

        kModulePositions = new Translation2d[] { kVehicleToBackRight, kVehicleToBackLeft, kVehicleToFrontLeft,
                kVehicleToFrontRight };

        // Pure Pursuit
        kPathFollowingMaxAccel = 80;
        kPathFollowingMaxVel = 130;

        kMinLookAhead = 12.0; // inches
        kMinLookAheadSpeed = 12.0; // inches per second
        kMaxLookAhead = 48.0; // inches
        kMaxLookAheadSpeed = kPathFollowingMaxVel; // inches per second

        kPurePursuitV = 1 / kPathFollowingMaxVel;
        kPurePursuitP = 0.0;

        // Limelight
        // To calculate these values, graph d = h/tan(ty + verticalAngle) + 89.625 for
        // multiple data points and graph on desmos to find intersection.

        limelightVerticalAngle = 18.8; // degrees
        limelightFloorClearance = 33.0; // inches
        magnifyThreshold = 0.0;

        // Targeting
        visionTargetingHeadingKp = 0.016;
        visionTargetingHeadingMinPower = 0.12;
        visionTargetingHeadingTxTolerance = 4;

        visionTargetingStrafeKp = 0.15;
        visionTargetingStrafeMinPower = 0.08;
        visionTargetingStrafeTxTolerance = 2;

        visionTargetingRangeKp = 0.015;
        visionTargetingRangeMinPower = 0.07;
        visionTargetingRangeTolerance = 4;

        visionTranslationalPID = new SwerveTranslationalPID(new FrogPIDF(1.0, 0.0, 0.0, ControlMode.Position_Control),
                (new FrogPIDF(1.0, 0.0, 0.0, ControlMode.Position_Control)));
        FrogPIDF multiuseheadingPID = new FrogPIDF(0.008, 0.000000001, 0.000001, 0.0, ControlMode.Position_Control);
        stabilizationPID = new FrogPIDF(0.015, 0.0, 0.0, 0.001, ControlMode.Position_Control);
        autuonomousAnglePID = multiuseheadingPID;
        snappingPID = stabilizationPID;
        visionAnglePID = multiuseheadingPID;
        kSwerveModuleGearRatio = 1.0 / 5.67;

        // Swerve module calculation positions (weird cooridate system that we just kind
        // of have to deal with for CoR bs)

        // @formatter:off
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

        double component = kWheelbaseLength / 2;
        kFrontLeftRadius = new Translation2d(component, component);
        kFrontRightRadius = new Translation2d(component, -component);
        kBackRightRadius = new Translation2d(-component, -component);
        kBackLeftRadius = new Translation2d(-component, component);

        // Gamespec

        // Shooter
        shooterMasterID = 13;
        shooterFollowerID = 2;

        // rightClimberID = 3;
        // leftClimberID = 18;

        // Shooter PID
        kShooterNeoKP = 0.0032 * (12.0 / 60.0);
        kShooterNeoKI = 0.000001;
        kShooterNeoKD = 0.0;
        kShooterNeoKS = 0.521 * (12.0 / 60.0);
        kShooterNeoKV = 0.00019;
        kShooterNeoKA = 0.141 * (12.0 / 60.0);
        kShooterNeoIZone = 0;

        kFlywheelRevsPerMotor = 2.0;
        kFlywheelIdleVelocity = 2000;

        // Spindexer
        spindexerID = 9;
        limitID = 0;

        kSpindexerTargetRpm = 55.0;
        kSpindexerRevsPerMotor = 1 / 144.0;
        kSpindexerIntakePos = 0.0;
        kSpindexerPosMaxVel = 0.0;
        kSpindexerPosMaxAccel = 0.0;

        // Spindexer PID
        kSpindexerP = 0.0006;
        kSpindexerI = 0.0;
        kSpindexerD = 0.0;
        kSpindexerS = 0.0;
        kSpindexerV = 0.0010666667;
        kSpindexerA = 0.00341;

        kSpindexerPosP = 0.0;
        kSpindexerPosI = 0.0;
        kSpindexerPosD = 0.0;
        kSpindexerPosF = 0.0;

        // Intake
        intakeID = 6;
        shiftForwardID = 5;
        shiftReverseID = 3;

        // Panel Spinner
        panelSpinnerID = 27;
        panelSpinnerSolenoidID = 4;

        // climberForwardID = 1;
        // climberReverseID = 0;

        // Hood
        hoodID = 7;
        hasHoodSolenoid = true;

    }

    @Override
    public boolean hasMotorizedHood() {
        return false;
    }

}
