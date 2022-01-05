/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;

import java.util.function.Supplier;

public class SwerveDriveCharacterize extends CommandBase {

    NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

    double priorAutospeed = 0;
    Number[] numberArray = new Number[10];

    double gearing, wheelDiameter;

    Supplier<Double> leftEncoderPosition;
    Supplier<Double> leftEncoderRate;
    Supplier<Double> rightEncoderPosition;
    Supplier<Double> rightEncoderRate;
    Supplier<Double> gyroAngleRadians;

    /**
     * Creates a new SwerveDriveCharacterize.
     */
    public SwerveDriveCharacterize(double gearing, double wheelDiameter) {
        this.gearing = gearing;
        this.wheelDiameter = wheelDiameter;

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset encoders
        Swerve.getInstance().resetDriveEncoder();

        leftEncoderPosition = () -> (Swerve.getInstance().getModules().get(1).getDriveMotorPosition() + Swerve.getInstance().getModules().get(2).getDriveMotorPosition()) / 2;
        leftEncoderRate = () -> (Swerve.getInstance().getModules().get(1).getDriveMotorVelocity() + Swerve.getInstance().getModules().get(1).getDriveMotorVelocity()) / 2;

        rightEncoderPosition = () -> (Swerve.getInstance().getModules().get(0).getDriveMotorPosition() + Swerve.getInstance().getModules().get(4).getDriveMotorPosition()) / 2;
        rightEncoderRate = () -> (Swerve.getInstance().getModules().get(0).getDriveMotorVelocity() + Swerve.getInstance().getModules().get(4).getDriveMotorVelocity()) / 2;

        gyroAngleRadians = () -> 0.0;

        // Set the update rate instead of using flush because of a ntcore bug
        // -> probably don't want to do this on a robot in competition
        NetworkTableInstance.getDefault().setUpdateRate(0.010);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double now = Timer.getFPGATimestamp();

        double leftPosition = leftEncoderPosition.get();
        double leftRate = leftEncoderRate.get();

        double rightPosition = rightEncoderPosition.get();
        double rightRate = rightEncoderRate.get();

        double battery = RobotController.getBatteryVoltage();

        double leftMotorVolts = (Swerve.getInstance().getModules().get(1).getDriveMotorVoltage() + Swerve.getInstance().getModules().get(2).getDriveMotorVoltage()) / 2;

        double rightMotorVolts = (Swerve.getInstance().getModules().get(0).getDriveMotorVoltage() + Swerve.getInstance().getModules().get(4).getDriveMotorVoltage()) / 2;

        double autospeed = autoSpeedEntry.getDouble(0);
        priorAutospeed = autospeed;

        Swerve.getInstance().drive(0, autospeed, 0, false);

        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = leftMotorVolts;
        numberArray[4] = rightMotorVolts;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        numberArray[9] = gyroAngleRadians.get();

        telemetryEntry.setNumberArray(numberArray);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
