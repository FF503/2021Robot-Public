package org.frogforce503.robot2021.paths;

import org.frogforce503.lib.control.*;
import org.frogforce503.lib.math.Rotation2;
import org.frogforce503.lib.math.Vector2;

import java.util.List;

/**
 * Class used to convert a list of Waypoints into a Path object consisting of
 * arc and line PathSegments
 *
 * @see Waypoint
 * @see Path
 */
public class TrajectoryBuilder {
    private static final ITrajectoryConstraint[] constraints = {new MaxVelocityConstraint(120.0),
            new MaxAccelerationConstraint(3.0 * 12.0), new CentripetalAccelerationConstraint(25.0 * 12.0)};

    public static Trajectory buildTrajectoryFromWaypoints(List<Vector2> w, PathContainer.EndpointState startState,
                                                          PathContainer.EndpointState endState, double snapDelay, boolean visionHeading, String name) {
        return getTrajectory(w, startState, endState, snapDelay, visionHeading, name, constraints);
    }

    private static Trajectory getTrajectory(List<Vector2> w, PathContainer.EndpointState startState, PathContainer.EndpointState endState, double snapDelay, boolean visionHeading, String name, ITrajectoryConstraint[] constraints) {
        Path path = new Path(Rotation2.fromDegrees(startState.getHeading()));

        for (int idx = 0; idx < w.size() - 1; idx++) {
            path.addSegment(new PathLineSegment(w.get(idx), w.get(idx + 1)));
        }

        return new Trajectory(startState.getVelocity(), endState.getVelocity(), endState.getHeading(), snapDelay, name,
                visionHeading, path, constraints);
    }

    public static Trajectory buildTrajectoryFromWaypoints(List<Vector2> w, PathContainer.EndpointState startState,
                                                          PathContainer.EndpointState endState, double snapDelay, boolean visionHeading, String name,
                                                          ITrajectoryConstraint[] constraintsOverride) {
        return getTrajectory(w, startState, endState, snapDelay, visionHeading, name, constraintsOverride);
    }
}
