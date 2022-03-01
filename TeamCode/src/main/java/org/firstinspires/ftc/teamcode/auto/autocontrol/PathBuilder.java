package org.firstinspires.ftc.teamcode.auto.autocontrol;

import java.util.ArrayList;

public class PathBuilder {
    private ArrayList<Waypoint> waypoints;

    public PathBuilder setStartingCoordinates(double x, double y, double heading) {
        waypoints.add(new Waypoint(x, y, heading, 0));
        return this;
    }

    public PathBuilder addWaypoint(double x, double y, double heading, double endVelocity) {
        waypoints.add(new Waypoint(x, y, heading, endVelocity));
        return this;
    }

    public Path build() {
        return new Path(waypoints);
    }
}
