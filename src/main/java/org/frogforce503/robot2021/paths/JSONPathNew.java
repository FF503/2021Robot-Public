package org.frogforce503.robot2021.paths;

import org.frogforce503.lib.control.CentripetalAccelerationConstraint;
import org.frogforce503.lib.control.ITrajectoryConstraint;
import org.frogforce503.lib.control.MaxAccelerationConstraint;
import org.frogforce503.lib.control.MaxVelocityConstraint;
import org.frogforce503.lib.math.Vector2;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import java.util.Arrays;
import java.util.List;

public class JSONPathNew extends PathContainer {

    private String name;

    private double startVelocity, startHeading, endVelocity, endHeading, snapDelay, maxVelocity, maxAcceleration,
            maxCentripetalAcceleration;
    private boolean lineUpWithVision, overrideConstraints;

    private List<Vector2> waypointList;

    public JSONPathNew(JSONObject object) {

        for (int pathIndex = 0; pathIndex < object.size(); pathIndex++) {

            JSONArray waypoints = (JSONArray) object.get("waypoints");
            JSONObject startState = (JSONObject) object.get("startState");
            JSONObject endState = (JSONObject) object.get("endState");
            JSONObject constraints = (JSONObject) object.get("constraints");

            Vector2[] temp = new Vector2[waypoints.size()];

            name = (String) object.get("name");
            startVelocity = (Double) startState.get("velocity");
            startHeading = (Double) startState.get("heading");
            endVelocity = (Double) endState.get("velocity");
            endHeading = (Double) endState.get("heading");
            snapDelay = (Double) object.get("snapDelay");
            lineUpWithVision = (Boolean) object.get("lineUpWithVision");
            overrideConstraints = (Boolean) constraints.get("overrideConstraints");
            maxVelocity = (Double) constraints.get("maxVelocity");
            maxAcceleration = (Double) constraints.get("maxAcceleration");
            maxCentripetalAcceleration = (Double) constraints.get("maxCentripetalAcceleration");

            for (int pointIndex = 0; pointIndex < waypoints.size(); pointIndex++) {
                JSONObject waypoint = (JSONObject) waypoints.get(pointIndex);
                Vector2 vector = new Vector2((Double) waypoint.get("x"), (Double) waypoint.get("y"));
                temp[pointIndex] = vector;

            }

            waypointList = Arrays.asList(temp);

        }
    }

    public void print() {
        System.out.println("==========================");
        System.out.println(name);
        System.out.println("==========================");

        System.out.println(waypointList.toString() + " < Waypoints");

        System.out.println(startVelocity + " < Starting Velocity");
        System.out.println(startHeading + " < Starting Heading");
        System.out.println(endVelocity + " < Ending Velocity");
        System.out.println(endHeading + " < Ending Heading");

        System.out.println(snapDelay + " < Snap Delay");
        System.out.println(lineUpWithVision + " < Vision Line Up");

    }

    @Override
    public List<Vector2> waypoints() {
        return waypointList;
    }

    @Override
    public EndpointState startState() {

        return new EndpointState(startVelocity, startHeading);
    }

    @Override
    public EndpointState endState() {
        return new EndpointState(endVelocity, endHeading);
    }

    @Override
    public double snapDelay() {
        return snapDelay;
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public boolean visionHeading() {
        return lineUpWithVision;
    }

    @Override
    public boolean overrideConstraints() {
        return overrideConstraints;
    }

    @Override
    public ITrajectoryConstraint[] constraints() {
        return new ITrajectoryConstraint[]{
                new MaxVelocityConstraint(maxVelocity),
                new MaxAccelerationConstraint(maxAcceleration),
                new CentripetalAccelerationConstraint(maxCentripetalAcceleration)
        };
    }
}