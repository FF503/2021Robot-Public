package org.frogforce503.lib.math;

import org.frogforce503.lib.util.Interpolable;

import java.io.Serializable;

/**
 * A rigid transform is a type of transformation that represents both a
 * translation and a rotation.
 *
 * @since 0.2
 */
public final class RigidTransform2 implements Serializable, Interpolable<RigidTransform2> {
    private static final long serialVersionUID = 1701732846641084965L;

    /**
     * The translation of the transform
     *
     * @since 0.2
     */
    public final Vector2 translation;

    /**
     * The rotation of the transform
     *
     * @since 0.2
     */
    public final Rotation2 rotation;

    /**
     * Create a new rigid transform from a translation and a rotation.
     *
     * @param translation The translation
     * @param rotation    The rotation
     * @since 0.2
     */
    public RigidTransform2(Vector2 translation, Rotation2 rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    private static Vector2 intersectionInternal(RigidTransform2 a, RigidTransform2 b) {
        final double t = ((a.translation.x - b.translation.x) * b.rotation.tan + b.translation.y - a.translation.y)
                / (a.rotation.sin - a.rotation.cos * b.rotation.tan);
        return a.translation.add(Vector2.fromAngle(a.rotation).scale(t));
    }

    /**
     * Gets the point of intersection between this rigid transform and another.
     *
     * @param other The other rigid transform
     * @return The point of intersection between the two transforms
     * @since 0.2
     */
    public Vector2 intersection(RigidTransform2 other) {
        if (rotation.isParallel(other.rotation)) {
            return new Vector2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        if (Math.abs(rotation.cos) < Math.abs(other.rotation.cos)) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "{T: " + translation + ", R: " + rotation + "}";
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof RigidTransform2)) {
            return false;
        }

        RigidTransform2 other = (RigidTransform2) obj;

        return translation.equals(other.translation) && rotation.equals(other.rotation);
    }

    @Override
    public RigidTransform2 interpolate(RigidTransform2 other, double t) {
        return new RigidTransform2(translation.interpolate(other.translation, t),
                rotation.interpolate(other.rotation, t));
    }
}
