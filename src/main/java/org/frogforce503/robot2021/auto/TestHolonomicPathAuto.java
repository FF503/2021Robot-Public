// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2021.auto;

import org.frogforce503.robot2021.commands.SwerveTrajectoryWithHeadingCommand;
import org.frogforce503.robot2021.subsystems.swerve.Pigeon;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestHolonomicPathAuto extends SequentialCommandGroup {
    /**
     * Creates a new StraightLineTest.
     */
    public TestHolonomicPathAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        // @formatter:off
        addCommands(
                new InstantCommand(Pigeon.getInstance()::setReversed),
                new SwerveTrajectoryWithHeadingCommand("straight_test_path"),
                new InstantCommand(Swerve.getInstance()::stop)
        );
        // @formatter:on

    }
}
