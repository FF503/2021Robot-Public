package org.frogforce503.robot2021.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frogforce503.robot2021.subsystems.Turret;

public class VisionTargetCommand extends CommandBase {
    /**
     * Creates a new ToggleSpindexerCommand.
     */
    double target;

    public VisionTargetCommand(double target) {
        this.target = target;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Turret.getInstance().setOverrideSkew(true);
        Turret.getInstance().setVisionTarget(this.target);
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
