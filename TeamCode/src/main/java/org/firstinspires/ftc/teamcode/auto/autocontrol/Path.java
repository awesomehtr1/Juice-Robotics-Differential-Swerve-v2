package org.firstinspires.ftc.teamcode.auto.autocontrol;

import java.util.ArrayList;

public class Path {
    private ArrayList<Waypoint> waypoints;
    private int currentSegment;

    public Path(ArrayList<Waypoint> waypoints) {
        this.waypoints = waypoints;
        currentSegment = 0;
    }

    public Waypoint getSegmentStart() {
        return waypoints.get(currentSegment);
    }

    public Waypoint getSegmentEnd() {
        return waypoints.get(currentSegment + 1);
    }

    public void advanceSegment() {
        if (currentSegment + 1 < waypoints.size())
            currentSegment += 1;
    }
}
