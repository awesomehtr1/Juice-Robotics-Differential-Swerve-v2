package org.firstinspires.ftc.teamcode.auto.autocontrol;

import java.util.ArrayList;

public class PathBuilder {
    private ArrayList<Waypoint> waypoints;

    public void setStartingCoordinates(double[] startingCoordinates, double heading) {
        waypoints.add(new Waypoint(startingCoordinates[0], startingCoordinates[1], heading, 0));
    }

    public void addWaypoint(double[] coordinates, double heading, double endVelocity) {
        waypoints.add(new Waypoint(coordinates[0], coordinates[1], heading, endVelocity));
    }

    public Path build() {
        return new Path(waypoints);
    }
}
