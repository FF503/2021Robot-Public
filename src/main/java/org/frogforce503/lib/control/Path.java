package org.frogforce503.lib.control;

import org.frogforce503.lib.math.Rotation2;
import org.frogforce503.lib.math.Vector2;
import org.frogforce503.lib.util.InterpolatingDouble;
import org.frogforce503.lib.util.InterpolatingTreeMap;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public final class Path implements Serializable {
    private static final long serialVersionUID = 6482549382610337817L;

    private final List<PathSegment> segments = new ArrayList<>();
    private final List<Double> distancesFromStart = new ArrayList<>();

    private final InterpolatingTreeMap<InterpolatingDouble, Rotation2> rotationAtDistance = new InterpolatingTreeMap<>();

    private double length = 0.0;

    public Path(Rotation2 startRotation) {
        rotationAtDistance.put(new InterpolatingDouble(0.0), startRotation);
    }

    public void addSegment(PathSegment segment) {
        segments.add(segment);
        distancesFromStart.add(length);
        length += segment.getLength();
    }

    public List<PathSegment> getSegments() {
        return segments;
    }

    public double getDistanceToSegmentStart(int segment) {
        return distancesFromStart.get(segment);
    }

    public double getDistanceToSegmentEnd(int segment) {
        return distancesFromStart.get(segment) + segments.get(segment).getLength();
    }

    private int getSegmentAtDistance(double distance) {
        int start = 0;
        int end = segments.size() - 1;
        int mid = start + (end - start) / 2;

        while (start < end) {
            mid = start + (end - start) / 2;

            if (distance > getDistanceToSegmentEnd(mid)) {
                start = mid + 1;
            } else if (distance < getDistanceToSegmentStart(mid)) {
                end = mid;
            } else {
                break;
            }
        }

        return mid;
    }

    public Vector2 getPositionAtDistance(double distance) {
        int currentSegment = getSegmentAtDistance(distance);
        double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

        return segments.get(currentSegment).getPositionAtDistance(segmentDistance);
    }

    public Rotation2 getHeadingAtDistance(double distance) {
        int currentSegment = getSegmentAtDistance(distance);
        double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

        return segments.get(currentSegment).getHeadingAtDistance(segmentDistance);
    }

    public Rotation2 getRotationAtDistance(double distance) {
        return rotationAtDistance.getInterpolated(new InterpolatingDouble(distance));
    }

    public double getLength() {
        return length;
    }
}
