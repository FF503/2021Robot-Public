package org.frogforce503.robot2021;

public class Constants {


    // Telemetry referenced in spindexer and shooter
    public static final boolean kOutputTelemetry = true;

    public static final double HEIGHT_OF_OUTER_PORT = 98.25 - 8.625;
    public static final double DEPTH_OF_OUTER_PORT = 29.25;


    // Gamespec
    // Shooter constants
    public static final int kShooterCurrentLimit = 60;
    public static final int kShooterStopOnTargetRpm = 150;
    public static final int kShooterStartOnTargetRpm = 50;
    public static final int kShooterMinOnTargetSamples = 20;

    // Spindexer constants
    public static final int kSpindexerCurrentLim = 80;
    public static final double kSpindexerLimitCounts = 250; // (200 / 1.5)

    // Intake constants
    public static final int kIntakeCurrentLimit = 60;

}