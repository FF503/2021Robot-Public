/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021;

import java.util.Arrays;

import org.frogforce503.robot2021.RobotState.GameState;
import org.frogforce503.robot2021.auto.AutoChooser;
import org.frogforce503.robot2021.loops.FroggyPoseController;
import org.frogforce503.robot2021.subsystems.Climber;
import org.frogforce503.robot2021.subsystems.Feeder;
import org.frogforce503.robot2021.subsystems.Hood;
import org.frogforce503.robot2021.subsystems.LegacyShooter;
import org.frogforce503.robot2021.subsystems.Shooter;
import org.frogforce503.robot2021.subsystems.Spindexer;
import org.frogforce503.robot2021.subsystems.Turret;
import org.frogforce503.robot2021.subsystems.intake.Intake;
import org.frogforce503.robot2021.subsystems.swerve.Pigeon;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.subsystems.vision.LimelightProcessor;
import org.frogforce503.robot2021.tests.RobotTest;
import org.frogforce503.robot2021.tests.TestInput;
import org.frogforce503.robot2021.tests.TestManager;
import org.frogforce503.robot2021.tests.modes.AlignWheels;
import org.frogforce503.robot2021.tests.modes.ReZeroSwerve;
import org.frogforce503.robot2021.tests.modes.StepSwerve;
import org.frogforce503.robot2021.tests.modes.SwerveSpin;
import org.json.simple.JSONObject;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    public static RobotHardware bot;
    AutoChooser autoChooser;
    Compressor c;

    private double mDisabledTime = 0.0;

    NetworkTableInstance networkTableInstance;
    NetworkTable testTable;
    SendableChooser<String> testChooser;
    TestManager testManager = new TestManager(new AlignWheels(), new SwerveSpin(), new ReZeroSwerve(),
            new StepSwerve());

    public Robot() {
        addPeriodic(() -> {
            if (Robot.bot.hasMotorizedHood()) {
                Turret.getInstance().readPeriodicInputs();
                Turret.getInstance().writePeriodicOutputs();

                Hood.getInstance().readPeriodicInputs();
                Hood.getInstance().writePeriodicOutputs();
            }

        }, 0.005, 0.005);
        addPeriodic(() -> {
            FroggyPoseController.updateOdometry();
            Swerve.getInstance().control();
        }, 0.010, 0.010);
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    @SuppressWarnings("unchecked")
    public void robotInit() {

        // Initialize bot
        RobotState.getInstance().setCurrentRobot(RobotState.Bot.CompBot);
        bot = RobotHardware.getInstance();

        OI.initialize();

        Swerve.getInstance().zeroSensors();
        Swerve.getInstance().coastAll();

        c = new Compressor(0);

        // CameraServer.getInstance().startAutomaticCapture().setResolution(240, 180);

        // autoChooser = new AutoChooser();
        if (Robot.bot.hasMotorizedHood())
            Pigeon.getInstance().setReversed();
        else
            Pigeon.getInstance().zero();

        FroggyPoseController.resetPose(new Pose2d());

        LiveWindow.disableAllTelemetry();

        // initialize everything

        if (Robot.bot.hasMotorizedHood()) {
            Hood.getInstance();
            Turret.getInstance();
            Shooter.getInstance();
        } else {
            // LegacyShooter.getInstance();
        }

        Intake.getInstance();

        LimelightProcessor.getInstance();
        Spindexer.getInstance();
        // autoChooser.resetClass();

        // send test info
        networkTableInstance = NetworkTableInstance.getDefault();
        testTable = networkTableInstance.getTable("testModes");

        testTable.getEntry("testList").setStringArray(testManager.getTestList());

        // set default test info
        testTable.getEntry("enabled").setBoolean(false);
        if (!Arrays.asList(testManager.getTestList()).contains(testTable.getEntry("selectedTest").getString("")))
            testTable.getEntry("selectedTest").setString(testManager.getDefault());

        // send test input info
        JSONObject testsObject = new JSONObject();

        for (RobotTest test : testManager.getTests()) {
            TestInput<Object>[] inputs = test.getInputs();
            JSONObject inputsObject = new JSONObject();

            for (TestInput<Object> input : inputs) {
                inputsObject.put(input.name, input.getType());
            }

            if (inputs.length > 0)
                testsObject.put(test.getName(), inputsObject);
        }

        testTable.getEntry("test_input_info").setString(testsObject.toJSONString());
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (Robot.bot.hasMotorizedHood()) {
            Hood.getInstance().readPeriodicInputs();
            Feeder.getInstance().outputTelemetry();

            Hood.getInstance().outputTelemetry();
            Turret.getInstance().outputTelemetry();
            Feeder.getInstance().outputTelemetry();

            Shooter.getInstance().readPeriodicInputs();
            Shooter.getInstance().writePeriodicOutputs();
            Shooter.getInstance().outputTelemetry();
        } else {
            LegacyShooter.getInstance().readPeriodicInputs();
            LegacyShooter.getInstance().outputTelemetry();
        }

        Intake.getInstance().readPeriodicInputs();
        Intake.getInstance().outputTelemetry();

        // Spindexer.getInstance().readPeriodicInputs();
        // Feeder.getInstance().readPeriodicInputs();

        Swerve.getInstance().outputTelemetry();
        Spindexer.getInstance().outputTelemetry();

        FroggyPoseController.outputPoseToDashboard();
    }

    @Override
    public void disabledInit() {
        RobotState.getInstance().setGameState(RobotState.GameState.DISABLED);
        Swerve.getInstance().updateDriverTranslation(new Translation2d());
        Swerve.getInstance().stop();
        Swerve.getInstance().coastAll();

        if (Robot.bot.hasMotorizedHood()) {
            Turret.getInstance().setBrakeMode(false);
            Hood.getInstance().stop();
            Feeder.getInstance().stop();
            Shooter.getInstance().stop();
        } else {
            LimelightProcessor.getInstance().dropLimelight();
        }

        Intake.getInstance().stop();
        if (Intake.getInstance().getState() != Intake.IntakeControlState.OFF) {
            Intake.getInstance().conformToState(Intake.IntakeControlState.OUT);
        }

        Spindexer.getInstance().setBrakeMode(false);
        Spindexer.getInstance().stop();

        mDisabledTime = Timer.getFPGATimestamp();
    }

    @Override
    public void disabledPeriodic() {

        // Wait for two seconds to brake then put drive in coast mode once
        if (Timer.getFPGATimestamp() - mDisabledTime >= 2 && !Swerve.getInstance().getCoasted()) {
            Swerve.getInstance().coastAll();
        }
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */

    @Override
    public void autonomousInit() {
        RobotState.getInstance().setGameState(RobotState.GameState.AUTON);

        Spindexer.getInstance().setBrakeMode(true);

        if (Robot.bot.hasMotorizedHood()) {
            Turret.getInstance().setBrakeMode(true);
            Turret.getInstance().lockOnTarget();
        }

        Swerve.getInstance().brakeAll();

        c.start();

        autoChooser.runSelectedAuto();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();

        Spindexer.getInstance().writePeriodicOutputs();
        Intake.getInstance().writePeriodicOutputs();

        if (Robot.bot.hasMotorizedHood()) {
            Feeder.getInstance().writePeriodicOutputs();
            Hood.getInstance().writePeriodicOutputs();
            Shooter.getInstance().writePeriodicOutputs();
        }
    }

    @Override
    public void teleopInit() {
        OI.flushJoystickCaches();

        RobotState.getInstance().setGameState(RobotState.GameState.TELEOP);

        // Initialize swerve stabilization to avoid sudden undesired movement
        Swerve.getInstance().initializeSwerveStabilization();
        Swerve.getInstance().brakeAll();

        // Initialize superstructure configuration
        Spindexer.getInstance().setBrakeMode(true);

        if (Robot.bot.hasMotorizedHood()) {
            Turret.getInstance().setBrakeMode(true);
            Turret.getInstance().moveToTarget();

            Hood.getInstance().setEncoderSetpoint(0);

            Turret.getInstance().setOverrideSkew(true);
            Turret.getInstance().setVisionTarget(0.0);
        }

        c.start();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        CommandScheduler.getInstance().run();

        // Specify enabled inputs
        oneControllerMode();

        Spindexer.getInstance().writePeriodicOutputs();
        // Swerve kinematics
        Swerve.getInstance().control();
        Swerve.getInstance().outputTelemetry();
        Spindexer.getInstance().writePeriodicOutputs();

        if (Robot.bot.hasMotorizedHood()) {
            Hood.getInstance().writePeriodicOutputs();
            Feeder.getInstance().writePeriodicOutputs();
        } else {
            // LegacyShooter.getInstance().readPeriodicInputs();
            LegacyShooter.getInstance().writePeriodicOutputs();
        }

        Intake.getInstance().writePeriodicOutputs();
    }

    // ONLY Driver input enabled
    private void oneControllerMode() {
        final double xTranslation = OI.getDriverLeftXValue();
        final double yTranslation = -OI.getDriverLeftYValue();
        Translation2d inputTranslation = new Translation2d(xTranslation, yTranslation);
        double rotInput = OI.getDriverRightXValue();

        inputTranslation = (inputTranslation.getNorm() > Swerve.getInstance().DRIVER_INPUT_TOLERANCE_TRANSLATION)
                ? inputTranslation
                : new Translation2d();
        rotInput = (Math.abs(rotInput) > Swerve.getInstance().DRIVER_INPUT_TOLERANCE_ROTATION) ? rotInput : 0.0;

        boolean lowSpeed = (OI.getDriverRightBumper());
        inputTranslation = lowSpeed ? inputTranslation.times(0.2) : inputTranslation;

        rotInput = Math.min(rotInput, 0.6);
        rotInput = Math.max(rotInput, -0.6);
        rotInput /= lowSpeed ? 5 : 1.43;

        if (OI.getOperatorLeftTrigger()) {
            Climber.getInstance().setOpenLoop(-OI.getOperatorLeftYValue());
        } else {
            Climber.getInstance().setOpenLoop(0);
        }

        // if (!OI.getDriverRightTrigger()) {
        // Swerve.getInstance().setFieldCentric(!OI.getDriverLeftBumper());
        // }

        if ((inputTranslation.getNorm() < Swerve
                .getInstance().DRIVER_INPUT_TOLERANCE_TRANSLATION)/* || OI.getDriverRightTrigger() */) {
            Swerve.getInstance().noDriverTranslationInput();
        } else {
            Swerve.getInstance().updateDriverTranslation(inputTranslation);
        }

        if ((Math.abs(rotInput) < Swerve.getInstance().DRIVER_INPUT_TOLERANCE_ROTATION)
                && (Swerve.getInstance().getHeadingMode() == Swerve.HeadingControlMode.DRIVER_CONTROLLED)) {
            Swerve.getInstance().noDriverHeadingInput();
        } else if ((Math.abs(rotInput) > Swerve.getInstance().DRIVER_INPUT_TOLERANCE_ROTATION)) {
            Swerve.getInstance().updateDriverHeadingInput(rotInput);
        }

        // Driver button controls
        if (OI.getSnapToZeroButton()) {
            Swerve.getInstance().snapToAngle(0.0);
        } else if (OI.getDriverPOV() == 90) {
            Swerve.getInstance().snapToAngle(90.0);
        } else if (OI.getDriverPOV() == 180) {
            Swerve.getInstance().snapToAngle(180.0);
        } else if (OI.getDriverXButton()) {
            Swerve.getInstance().snapToAngle(270.0);
        } else if (OI.getClimbSnapButton()) {
            Swerve.getInstance().snapToAngle(67.5);
        } else if (OI.getAnchorButton()) {
            Swerve.getInstance().enableAnchorMode();
        } else {
            Swerve.getInstance().killAnchorMode();
        }
    }

    @Override
    public void testInit() {
        RobotState.getInstance().setGameState(RobotState.GameState.TEST);

        // networkTableInstance = NetworkTableInstance.getDefault(); // now done in
        // robot init
        // testTable = networkTableInstance.getTable("testModes"); // now done in robot
        // init

        // testChooser = new SendableChooser<>();

        // boolean firstDone = false;

        // tells dashboard what each test's inputs are and what type the input is,
        // example output:
        /**
         * { "test1": { "power": "Double" }, "test2": { "open": "Boolean", "mode":
         * "String" } }
         */

        // for (String testName : testManager.getTestList()) {
        // if (!firstDone)
        // testChooser.setDefaultOption(testName, testName);
        // else
        // testChooser.addOption(testName, testName);
        // }

        // SmartDashboard.putData(testChooser);

        // boolean testEnabled = false;
        // SmartDashboard.putBoolean("testEnabled", testEnabled);

        // testTable.getEntry("testList").setStringArray(testManager.getTestList()); //
        // now done in robot init

        TestManager.clearLog();

        Swerve.getInstance().brakeAll();

        networkTableInstance.startClientTeam(503);

        RobotState.getInstance().setGameState(GameState.TEST);

        c.start();
    }

    /**
     * This function is called periodically during test mode.
     */
    String testOutput = "Test Disabled";

    boolean testWasEnabled = false;

    @Override
    public void testPeriodic() {
        testManager.setActive(testTable.getEntry("selectedTest").getString(testManager.getDefault()));
        // TestInput<Object>[] n_inputs = testManager.getActive().getInputs();

        // testManager.setActive(testChooser.getSelected());

        boolean testEnabled = testTable.getEntry("enabled").getBoolean(false);
        // boolean testEnabled = SmartDashboard.getBoolean("testEnabled", false);
        // boolean testEnabled = false;

        if (testEnabled) {
            testManager.handleInputTable(testTable);

            if (testWasEnabled)
                testManager.init();

            // testManager.handleInput();
            testManager.run();
            String output = testManager.getOutput();
            if (!output.equals(testOutput))
                testOutput = output;

            // testManager.getActiveSubsystem().outputTelemetry();
            testWasEnabled = false;
        } else {
            if (!testOutput.equals("Test Disabled")) {
                testOutput = "Test Disabled";
                testManager.stop();
            }
            testWasEnabled = true;
        }

        testTable.getEntry("testOutput").setString(testOutput);
        testTable.getEntry("testLog").setString(testManager.getLog());
        SmartDashboard.putString("testOutput", testOutput);
    }
}
