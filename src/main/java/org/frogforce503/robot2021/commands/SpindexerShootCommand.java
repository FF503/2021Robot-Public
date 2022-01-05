// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2021.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frogforce503.robot2021.subsystems.Spindexer;

public class SpindexerShootCommand extends CommandBase {

    private final double rpm;

    /**
     * Creates a new ShooterVelocityCommand.
     */
    public SpindexerShootCommand(double rpm) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.rpm = rpm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Spindexer.getInstance().setShootState(this.rpm);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
