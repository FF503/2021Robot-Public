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

/*
 * @formatter:off
 *
 * LIMELIGHT ADDRESSES:
 * SHOOTER: http://10.5.2.11:5801 OR http://limelight.local:5801
 * FLOOR: http://10.5.2.12:5801 OR http://limelight-floor.local:5801
 *
 * Virtual wheelbase = 22in
 *
 * AutoNav:
 *
 * MAX VELOCITY = 150in/sec
 * MAX ACCELERATION = 130in/sec^2
 *
 * Galactic Search
 *
 * MAX VELOCITY = 130in/sec
 * MAX ACCELERATION = 65in/sec^2
 *
 * @formatter:on
 */

public class RobotHardwareCompBot extends RobotHardware { // 3 2

    /* All distance measurements are in inches, unless otherwise noted */

    @Override
    public void initializeConstants() {
        // Swerve Module JSON file names
        backLeftName = "CompBackLeft";
        backRightName = "CompBackRight";
        frontLeftName = "CompFrontLeft";
        frontRightName = "CompFrontRight";

        // Swerve Calculations Constants (measurements are in inches)
        kWheelbaseLength = 22;
        kWheelbaseWidth = 22;
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
        // multiple data points and graph on Desmos to find intersection.

        limelightVerticalAngle = 20.0; // degrees
        limelightFloorClearance = 24.0; // inches
        magnifyThreshold = 0.0;

        // Targeting
        visionTargetingHeadingKp = 0.04;
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
        snappingPID = new FrogPIDF(0.030, 0.0, 0.000001, 0.001, ControlMode.Position_Control);
        visionAnglePID = multiuseheadingPID;
        kSwerveModuleGearRatio = 1.0 / 5.67;

        // Swerve module calculation positions (weird coordinate system that we just
        // kind
        // of have to deal with for CoR bs)

        // @formatter:off
        /*
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

        kDriveNeoKP = 0.0017361 / 4;// 3.99E-10;
        kDriveNeoKS = 0.0772 / 12;
        kDriveNeoKV = 0.125 / 60 / 12;
        kDriveNeoKA = 0.00562 / 60 / 60 / 12;

        // Gamespec

        // Climber
        climberID = 12;

        // Shooter
        shooterMasterID = 2;
        shooterFollowerID = 13;

        // Shooter PID
        kShooterNeoKP = 0.000234;// 0.60 / 500;
        kShooterNeoKI = 0.0000;
        kShooterNeoKD = 0.0;
        kShooterNeoKS = 0.352 / 2.0;// 0.03583;
        kShooterNeoKV = 0.0000976388888888 * 2;// 0.0001840;
        kShooterNeoKA = 0.0731;
        kShooterNeoIZone = 0;

        kFlywheelRevsPerMotor = 2.0;
        kFlywheelIdleVelocity = 2000;

        // Spindexer
        spindexerID = 6;

        // Intake
        intakeID = 9;
        shiftForwardID = 0;
        shiftReverseID = 1;

        // Turret
        turretID = 7;
        kTurretMaxSpeed = 400;
        kTurretStartingAngle = 0;

        // Hood
        leftHoodID = 0;
        rightHoodID = 1;
        hoodEncoderID = 23;

        // Hood PIDcos
        kHoodHorizontalOffset = 27;
        kHoodP = 0.04;
        kHoodI = 0;
        kHoodD = 0.01;// 0.001;
        kHoodCos = 0.055;
        kHoodStatic = 0.0;

        // Panel Spinner
        panelSpinnerID = 27;
        panelSpinnerSolenoidID = 4;

        climberForwardID = 2;
        climberReverseID = 3;

    }
}
