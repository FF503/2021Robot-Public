package org.frogforce503.lib.control;

import org.frogforce503.lib.math.Rotation2;
import org.frogforce503.lib.math.Vector2;

import java.util.Objects;

public final class PathArcSegment extends PathSegment {
    private static final long serialVersionUID = 1817031344401689498L;

    private final Vector2 center;
    private final Vector2 deltaStart;
    private final Vector2 deltaEnd;
    private final boolean clockwise;

    public PathArcSegment(Vector2 start, Vector2 end, Vector2 center) {
        super(start, end);
        this.center = center;
        this.deltaStart = start.subtract(center);
        this.deltaEnd = end.subtract(center);

        clockwise = deltaStart.cross(deltaEnd) <= 0.0;
    }

    @Override
    public double getCurvature() {
        return 1.0 / deltaStart.length;
    }

    @Override
    public Vector2 getPositionAtPercentage(double percentage) {
        double deltaAngle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() * (clockwise ? -1.0 : 1.0)
                * percentage;
        return center.add(deltaStart.rotateBy(Rotation2.fromRadians(deltaAngle)));
    }

    @Override
    public Rotation2 getHeadingAtPercentage(double percentage) {
        double angle = Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians() * (clockwise ? -1.0 : 1.0) * percentage
                + (clockwise ? -0.5 * Math.PI : 0.5 * Math.PI); // Add or subtract 90 degrees to the angle based on the
        // direction of travel
        return deltaStart.rotateBy(Rotation2.fromRadians(angle)).getAngle();
    }

    @Override
    public double getLength() {
        return deltaStart.length * Vector2.getAngleBetween(deltaStart, deltaEnd).toRadians();
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof PathArcSegment) {
            PathArcSegment other = (PathArcSegment) o;
            return deltaStart.equals(other.deltaStart) && deltaEnd.equals(other.deltaEnd)
                    && center.equals(other.center);
        }

        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(center, deltaStart, deltaEnd);
    }

    @Override
    public String toString() {
        return String.format("{start: %s, end: %s, center: %s}", getStart(), getEnd(), center);
    }
}
