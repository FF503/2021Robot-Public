package org.frogforce503.robot2021.paths;

import org.frogforce503.lib.control.ITrajectoryConstraint;
import org.frogforce503.lib.control.Trajectory;
import org.frogforce503.lib.math.Vector2;

import java.util.List;

/**
 * Interface containing all information necessary for a path including the Path
 * itself, the Path's starting pose, and whether or not the robot should drive
 * in reverse along the path.
 */
public abstract class PathContainer {
    public Trajectory buildTrajectory() {
        if (overrideConstraints()) {
            return TrajectoryBuilder.buildTrajectoryFromWaypoints(waypoints(), startState(), endState(), snapDelay(),
                    visionHeading(), name(), constraints());
        }
        return TrajectoryBuilder.buildTrajectoryFromWaypoints(waypoints(), startState(), endState(), snapDelay(),
                visionHeading(), name());
    }

    public boolean overrideConstraints() {
        return false;
    }

    public ITrajectoryConstraint[] constraints() {
        return null;
    }

    public abstract List<Vector2> waypoints();

    public abstract EndpointState startState();

    public abstract EndpointState endState();

    public abstract double snapDelay();

    public String name() {
        return "unnamedPath";
    }

    public boolean visionHeading() {
        return false;
    }

    static class EndpointState {
        private final double mVelocity;
        private final double mHeading;

        public EndpointState(double velocity, double heading) {
            this.mVelocity = velocity;
            this.mHeading = heading;
        }

        public double getVelocity() {
            return mVelocity;
        }

        public double getHeading() {
            return mHeading;
        }
    }
}
