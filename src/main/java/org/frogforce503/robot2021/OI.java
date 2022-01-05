package org.frogforce503.robot2021;

import org.frogforce503.robot2021.commands.FeedCommand;
import org.frogforce503.robot2021.commands.HoodAngleCommand;
import org.frogforce503.robot2021.commands.HoodSetpointIncrementCommand;
import org.frogforce503.robot2021.commands.NoopCommand;
import org.frogforce503.robot2021.commands.ShooterVelocityCommand;
import org.frogforce503.robot2021.commands.ShooterVelocityIncrementCommand;
import org.frogforce503.robot2021.commands.SpindexerMixCommand;
import org.frogforce503.robot2021.commands.SpindexerShootCommand;
import org.frogforce503.robot2021.commands.ToggleEjectCommand;
import org.frogforce503.robot2021.commands.ToggleIntakeCommand;
import org.frogforce503.robot2021.commands.ToggleSpindexerCommand;
import org.frogforce503.robot2021.commands.ToggleVisionSnap;
import org.frogforce503.robot2021.commands.TurretAutoHomeCommand;
import org.frogforce503.robot2021.commands.VisionStrafeCommand;
import org.frogforce503.robot2021.subsystems.Hood;
import org.frogforce503.robot2021.subsystems.LegacyShooter;
import org.frogforce503.robot2021.subsystems.Shooter;
import org.frogforce503.robot2021.subsystems.Spindexer;
import org.frogforce503.robot2021.subsystems.Turret;
import org.frogforce503.robot2021.subsystems.intake.Intake;
import org.frogforce503.robot2021.subsystems.swerve.Pigeon;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

@SuppressWarnings("unused")
public class OI {

    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());

    private static final XboxController driverJoystick = new XboxController(0);
    private static final XboxController operatorJoystick = new XboxController(1);

    private static final JoystickButton driverA = new JoystickButton(driverJoystick, 1);
    private static final JoystickButton driverB = new JoystickButton(driverJoystick, 2);
    private static final JoystickButton driverX = new JoystickButton(driverJoystick, 3);
    private static final JoystickButton driverY = new JoystickButton(driverJoystick, 4);

    private static final JoystickButton driverRB = new JoystickButton(driverJoystick, 6);
    private static final JoystickButton driverLB = new JoystickButton(driverJoystick, 5);

    private static final Trigger driverLT = new Trigger(OI::getDriverLeftTrigger);
    private static final Trigger driverRT = new Trigger(OI::getDriverRightTrigger);

    private static final JoystickButton driverBack = new JoystickButton(driverJoystick, 7);
    private static final JoystickButton driverStart = new JoystickButton(driverJoystick, 8);

    private static final JoystickButton operatorY = new JoystickButton(operatorJoystick, 4);
    private static final JoystickButton operatorX = new JoystickButton(operatorJoystick, 3);
    private static final JoystickButton operatorB = new JoystickButton(operatorJoystick, 2);
    private static final JoystickButton operatorA = new JoystickButton(operatorJoystick, 1);
    private static final JoystickButton operatorMenu = new JoystickButton(operatorJoystick, 8);
    private static final JoystickButton operatorSelect = new JoystickButton(operatorJoystick, 7);
    private static final JoystickButton operatorLB = new JoystickButton(operatorJoystick, 5);
    private static final JoystickButton operatorRB = new JoystickButton(operatorJoystick, 6);
    private static final JoystickButton operatorRJ = new JoystickButton(operatorJoystick, 10);

    private static final Trigger operatorLT = new Trigger(OI::getOperatorLeftTrigger);
    private static final Trigger operatorRT = new Trigger(OI::getOperatorRightTrigger);

    private static final Trigger operatorPOVUp = new Trigger(OI::getOperatorPOVForward);
    private static final Trigger operatorPOVLeft = new Trigger(OI::getOperatorPOVLeft);

    private static final Trigger turretCwLimit = new Trigger(() -> Turret.getInstance().getCwLimitPressed());
    private static final Trigger turretCcwLimit = new Trigger(() -> Turret.getInstance().getCcwLimitPressed());

    public static void initialize() {

        // @formatter:off

        /* DRIVER CONTROLS */

        // Shoot balls while driver holds down right bumper
        if (Robot.bot.hasMotorizedHood()) {
            driverRT.whenActive(
                    new FeedCommand(0.9).alongWith(
                            new SpindexerShootCommand(65.0),
                            new ConditionalCommand(
                                    new InstantCommand(() -> Intake.getInstance().conformToState(Intake.IntakeControlState.INTAKING)),
                                    new NoopCommand(),
                                    () -> Turret.getInstance().getCurrentState() != Turret.TurretControlState.POSITION
                            )
                    )
            );
            driverRT.whenInactive(
                    new FeedCommand(0.0).alongWith(
                            new InstantCommand(Spindexer.getInstance()::stop),
                            new InstantCommand(Shooter.getInstance()::stop),
                            new HoodAngleCommand(0.0),
                            new InstantCommand(Turret.getInstance()::moveToTarget),
                            new InstantCommand(() -> Intake.getInstance().conformToState(Intake.IntakeControlState.OFF))
                    )
            );
        } else {
            driverRT.whenActive(
                new InstantCommand(() -> Spindexer.getInstance().setOpenLoop(0.8))
                // new InstantCommand()
            );
            driverRT.whenInactive(
                new InstantCommand(Spindexer.getInstance()::stop)
            );
            
            driverRB.toggleWhenPressed(new ToggleVisionSnap());
            driverLB.toggleWhenPressed(new InstantCommand(() -> LimelightProcessor.getInstance().toggleLimelight()));

            driverLT.whenInactive(
                new InstantCommand(LegacyShooter.getInstance()::aimShooterSpeedWithVision)
            );

            driverBack.toggleWhenPressed(new VisionStrafeCommand());
        }

        driverB.whenReleased(new InstantCommand(Robot.bot.hasMotorizedHood() ? Pigeon.getInstance()::setReversed : Pigeon.getInstance()::zero));

        /* OPERATOR CONTROLS */

        // Drop and start intake/spindexer when driver presses X
        // operatorA.whenReleased(new ToggleIntakeCommand());

        operatorA.whenReleased(new ConditionalCommand(new ToggleIntakeCommand().alongWith(new SpindexerMixCommand()),
                new NoopCommand(), () -> Spindexer.getInstance().getState() != Spindexer.SpindexerControlState.SHOOTING));

        operatorB.whenReleased(new ToggleEjectCommand());

        operatorY.whenReleased(new ToggleSpindexerCommand());

        operatorLB.whenReleased(
                new ShooterVelocityCommand(1800).alongWith(
                        new HoodAngleCommand(0)
                )
        );

        operatorX.whenReleased(new InstantCommand(() ->  {
            // if (Robot.bot.hasMotorizedHood())
            Intake.getInstance().conformToState(Intake.IntakeControlState.OFF);
            // else
            //     LegacyIntake.getInstance().conformToState(LegacyIntake.IntakeControlState.OFF);
        }));

        operatorSelect.whenReleased(new HoodSetpointIncrementCommand());
        operatorMenu.whenReleased(new ShooterVelocityIncrementCommand());

        // Start tracking shooter and hood to vision
        operatorRT.whenInactive(
                Robot.bot.hasMotorizedHood() ?  
                new InstantCommand(() -> Hood.getInstance().aimHoodWithVision()).alongWith(
                        new InstantCommand(() -> Shooter.getInstance().aimShooterSpeedWithVision())
                ) :
                new InstantCommand(() -> LegacyShooter.getInstance().aimShooterSpeedWithVision())
        );

        if (Robot.bot.hasMotorizedHood()) {
            operatorPOVUp.whenActive(
                    new InstantCommand(() -> Turret.getInstance().setAngle(-180.0)).alongWith(
                            new ShooterVelocityCommand(1900),
                            new HoodAngleCommand(0)
                    )
            );
    
            operatorPOVLeft.whenActive(
                    new InstantCommand(() -> Turret.getInstance().setAngle(-90.0)).alongWith(
                            new ShooterVelocityCommand(1900),
                            new HoodAngleCommand(0)
                    )
            );
        }

        // Turret limit switch listeners to reset
        turretCwLimit.whenActive(() -> Turret.getInstance().resetEncoderZero(false));
        turretCcwLimit.whenActive(() -> Turret.getInstance().resetEncoderZero(true));

        operatorRB.whenReleased(new TurretAutoHomeCommand());

        // @formatter:on
    }

    public static boolean getAnchorButton() {
        return getDriverLeftTrigger();
    }

    public static boolean getDriverXButton() {
        return driverX.get();
    }

    public static boolean getDriverLeftBumper() {
        return driverLB.get();
    }

    public static boolean getDriverRightBumper() {
        return driverRB.get();
    }

    public static boolean getZeroGyroButton() {
        return driverJoystick.getBButton();
    }

    public static boolean stopShooterButtonReleased() {
        return operatorJoystick.getXButtonReleased();
    }

    public static boolean shortShotButtonReleased() {
        return operatorJoystick.getYButtonReleased();
    }

    public static boolean longShotButtonReleased() {
        return operatorJoystick.getAButtonReleased();
    }

    public static boolean toggleSpindexerButtonReleased() {
        return driverJoystick.getBButtonReleased();
    }

    public static boolean toggleIntakeButtonReleased() {
        return operatorJoystick.getBumperReleased(Hand.kRight);
    }

    public static boolean toggleEjectButtonReleased() {
        return operatorJoystick.getBumperReleased(Hand.kLeft);
    }

    public static boolean toggleIdleModeButtonReleased() {
        return operatorJoystick.getBButtonReleased();
    }

    public static boolean spindexerHomeButtonReleased() {
        return operatorJoystick.getBackButtonReleased();
    }

    public static boolean aimButtonPressed() {
        return driverRB.get();
    }

    public static boolean strafeButtonPressed() {
        return driverLB.get();
    }

    public static boolean getAimButtonPressed() {
        return driverJoystick.getBumperPressed(Hand.kRight);
    }

    public static boolean getStrafeButtonPressed() {
        return driverJoystick.getBumperPressed(Hand.kLeft);
    }

    public static boolean getAimButtonReleased() {
        return driverJoystick.getBumperReleased(Hand.kRight);
    }

    public static boolean getStrafeButtonReleased() {
        return driverJoystick.getBumperReleased(Hand.kLeft);
    }

    public static boolean getOverrideShootButtonReleased() {
        return driverJoystick.getBackButtonReleased();
    }

    public static double getDriverLeftYValue() {
        return driverJoystick.getRawAxis(1);
    }

    public static double getDriverLeftXValue() {
        return driverJoystick.getRawAxis(0);
    }

    public static double getDriverRightYValue() {
        return driverJoystick.getRawAxis(5);
    }

    public static double getDriverRightXValue() {
        return driverJoystick.getRawAxis(4);
    }

    public static boolean getClimbSnapButton() {
        return driverJoystick.getStartButton();
    }

    public static int getDriverPOV() {
        return driverJoystick.getPOV();
    }

    public static int getOperatorPOV() {
        return operatorJoystick.getPOV();
    }

    public static boolean getOperatorPOVForward() {
        return getOperatorPOV() == 0;
    }

    public static boolean getOperatorPOVLeft() {
        return getOperatorPOV() == 270;
    }

    public static boolean getDriverLeftTrigger() {
        return driverJoystick.getRawAxis(2) >= 0.5;
    }

    public static boolean getDriverRightTrigger() {
        return driverJoystick.getRawAxis(3) >= 0.5;
    }

    public static double getOperatorLeftYValue() {
        return operatorJoystick.getRawAxis(1);
    }

    public static double getOperatorRightYValue() {
        return operatorJoystick.getRawAxis(5);
    }

    public static boolean getOperatorLeftTrigger() {
        return operatorJoystick.getRawAxis(2) >= 0.5;
    }

    public static boolean getOperatorRightTrigger() {
        return operatorJoystick.getRawAxis(3) >= 0.5;
    }

    public static boolean getSnapToZeroButton() {
        return driverJoystick.getAButton();
    }

    public static boolean shooterIncrementPressed() {
        return getOperatorRightTrigger() && getOperatorPOV() == 0;
    }

    public static boolean shooterDecrementPressed() {
        return getOperatorRightTrigger() && getOperatorPOV() == 180;
    }

    public static JoystickButton getAimButton() {
        return driverRB;
    }

    public static boolean getShootButtonPressed() {
        return driverJoystick.getBumperPressed(Hand.kLeft);
    }

    public static boolean getShootButtonReleased() {
        return driverJoystick.getBumperReleased(Hand.kLeft);
    }

    public static void setDriveRumble(double rumble) {
        driverJoystick.setRumble(GenericHID.RumbleType.kLeftRumble, rumble);
        driverJoystick.setRumble(GenericHID.RumbleType.kRightRumble, rumble);
    }

    public static void flushJoystickCaches() {
        for (int button = 1; button <= 10; button++) {
            driverJoystick.getRawButtonPressed(button);
            driverJoystick.getRawButtonReleased(button);
            operatorJoystick.getRawButtonPressed(button);
            operatorJoystick.getRawButtonReleased(button);
        }
    }

}