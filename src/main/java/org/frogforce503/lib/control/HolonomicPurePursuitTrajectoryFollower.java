package org.frogforce503.lib.control;

import org.frogforce503.lib.math.RigidTransform2;
import org.frogforce503.lib.math.Vector2;
import org.frogforce503.lib.util.HolonomicDriveSignal;
import org.frogforce503.lib.util.HolonomicFeedforward;

import java.util.function.Function;

public class HolonomicPurePursuitTrajectoryFollower extends PurePursuitTrajectoryFollowerBase<HolonomicDriveSignal> {
    private final HolonomicFeedforward feedforward;

    private final PidController rotationController;

    public HolonomicPurePursuitTrajectoryFollower(Function<Vector2, Double> lookaheadDistanceFunction,
                                                  double finishRange, HolonomicFeedforward feedforward, PidConstants rotationPidConstants) {
        super(lookaheadDistanceFunction);
        this.feedforward = feedforward;

        this.rotationController = new PidController(rotationPidConstants);
        this.rotationController.setInputRange(0.0, 2.0 * Math.PI);
        this.rotationController.setContinuous(true);
    }

    public HolonomicPurePursuitTrajectoryFollower(double lookaheadDistance, double finishRange,
                                                  HolonomicFeedforward feedforward, PidConstants rotationPidConstants) {
        this(velocity -> lookaheadDistance, finishRange, feedforward, rotationPidConstants);
    }

    @Override
    protected HolonomicDriveSignal getDriveSignal(RigidTransform2 currentPose, Vector2 velocity,
                                                  double rotationalVelocity, Trajectory trajectory, Trajectory.Segment closestSegment,
                                                  Trajectory.Segment lookaheadSegment, PathSegment pursuitSegment, double dt) {
        Vector2 pursuitDelta = lookaheadSegment.translation.subtract(currentPose.translation);

        Vector2 velocityVector = pursuitDelta.scale(closestSegment.velocity / pursuitDelta.length);
        Vector2 accelerationVector = pursuitDelta.scale(closestSegment.acceleration / pursuitDelta.length);
        Vector2 translationalVelocity = feedforward.calculateFeedforward(velocityVector, accelerationVector);

        rotationController.setSetpoint(closestSegment.rotation.toRadians());

        return new HolonomicDriveSignal(translationalVelocity,
                rotationController.calculate(currentPose.rotation.toRadians(), dt), true);
    }

    @Override
    protected void reset() {
        super.reset();

        rotationController.reset();
    }
}
