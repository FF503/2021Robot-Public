package org.frogforce503.robot2021;

import org.frogforce503.robot2021.subsystems.swerve.Pigeon;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public class RobotState {
    private static final RobotState instance = new RobotState();
    private Bot currentRobot;
    private Pose2d currentPose = new Pose2d();
    private double currentTheta;
    private boolean autonReadyToShoot;
    private boolean indexerHomed = true;
    private boolean indexerPastEncoderCount = false;
    private boolean autonRequestsFasterHoming;
    private double autonRequestedPreHomingCounts;
    private double shooterSpeedIncrement = 1.0;
    private GameState gameState = GameState.DISABLED;
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    private double[] fieldCentricSpeeds = { 0, 0 };

    public static RobotState getInstance() {
        return instance;
    }

    public double getAutonRequestedPreHomingCounts() {
        return this.autonRequestedPreHomingCounts;
    }

    public void setAutonRequestedPreHomingCounts(double d) {
        this.autonRequestedPreHomingCounts = d;
    }

    public boolean getAutonRequestsFasterHoming() {
        return autonRequestsFasterHoming;
    }

    public void setAutonRequestsFasterHoming(boolean b) {
        this.autonRequestsFasterHoming = b;
    }

    public boolean isIndexerHomed() {
        return indexerHomed;
    }

    public void setIndexerHomed(boolean indexerHomed) {
        this.indexerHomed = indexerHomed;
    }

    public void incrementShooterSpeed(double i) {
        shooterSpeedIncrement += i;
    }

    public double getShooterSpeedIncrement() {
        return shooterSpeedIncrement;
    }

    public GameState getGameState() {
        return gameState;
    }

    public void setGameState(GameState gamestate) {
        this.gameState = gamestate;
    }

    public boolean isAutonReadyToShoot() {
        return autonReadyToShoot;
    }

    public void setAutonReadyToShoot(boolean ready) {
        this.autonReadyToShoot = ready;
    }

    public Bot getCurrentRobot() {
        return this.currentRobot;
    }

    public void setCurrentRobot(final Bot currentRobot) {
        this.currentRobot = currentRobot;
    }

    public synchronized Pose2d getCurrentPose() {
        return currentPose;
    }

    public synchronized ChassisSpeeds getCurrentSpeeds() {
        return currentSpeeds;
    }

    public synchronized double[] getFieldCentricSpeeds() {
        return fieldCentricSpeeds;
    }

    public synchronized void setCurrentPose(Pose2d currentPose) {
        this.currentPose = currentPose;
    }

    public synchronized void setCurrentSpeeds(ChassisSpeeds chassisSpeeds) {
        this.currentSpeeds = chassisSpeeds;
    }

    public synchronized void setFieldCentricSpeeds(double xVelocity, double yVelocity) {
        this.fieldCentricSpeeds[0] = xVelocity;
        this.fieldCentricSpeeds[1] = yVelocity;
    }

    public synchronized double getCurrentTheta() {
        currentTheta = Pigeon.getInstance().getYaw();
        return currentTheta;
    }

    public synchronized void setCurrentTheta(double currentTheta) {
        this.currentTheta = currentTheta;
    }

    public synchronized double getGyroOffset() {
        return 0.0;// this.startingDirection.getGyroOffset();
    }

    public boolean getIndexerPastEncoderCount() {
        return this.indexerPastEncoderCount;
    }

    public void setIndexerPastEncoderCount(boolean b) {
        this.indexerPastEncoderCount = b;
    }

    public enum GameState {
        AUTON, TELEOP, TEST, DISABLED
    }

    public enum Bot {
        Automatic, ProgrammingBot, CompBot, PracticeBot, LegacyBot
    }

}