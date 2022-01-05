package org.frogforce503.lib.control;

import org.frogforce503.lib.math.Rotation2;
import org.frogforce503.lib.math.Vector2;

public class Waypoint {
    public final Vector2 position;
    public final Rotation2 heading;
    public final Rotation2 rotation;

    public Waypoint(double x, double y) {
        this.position = new Vector2(x, y);
        this.heading = null;
        this.rotation = null;
    }
}
