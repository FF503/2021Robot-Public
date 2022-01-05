// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2021.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frogforce503.robot2021.subsystems.Shooter;

public class ShooterVelocityIncrementCommand extends CommandBase {

    public ShooterVelocityIncrementCommand() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Shooter.getInstance().setVelocity(Shooter.getInstance().getSetpointRpm() + 100);
    }

    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
