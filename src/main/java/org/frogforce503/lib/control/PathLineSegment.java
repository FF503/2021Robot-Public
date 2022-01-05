package org.frogforce503.lib.control;

import org.frogforce503.lib.math.Rotation2;
import org.frogforce503.lib.math.Vector2;

import java.util.Objects;

public final class PathLineSegment extends PathSegment {
    private static final long serialVersionUID = -5561222714094182439L;

    private final Vector2 delta;

    public PathLineSegment(Vector2 start, Vector2 end) {
        super(start, end);
        this.delta = end.subtract(start);
    }

    @Override
    public double getCurvature() {
        return 0;
    }

    @Override
    public Vector2 getPositionAtPercentage(double percentage) {
        return getStart().add(delta.scale(percentage));
    }

    @Override
    public Rotation2 getHeadingAtPercentage(double percentage) {
        return delta.getAngle();
    }

    @Override
    public double getLength() {
        return delta.length;
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof PathLineSegment) {
            PathLineSegment other = (PathLineSegment) o;
            return getStart().equals(other.getStart()) && delta.equals(other.delta);
        }

        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getStart(), delta);
    }

    @Override
    public String toString() {
        return String.format("{start: %s, end: %s}", getStart(), getEnd());
    }
}
