package org.firstinspires.ftc.teamcode.swerve;

public class SwerveKinematics {
    // trackwidth: horizontal distance between center of pairs of wheels
    // wheelbase: vertical distance between center of pairs of wheels
    private double trackwidth, wheelbase;

    // 2 dimensional array storing the angle and speed of all modules
    // follows RF, LF, LB, RB order
    // same order as quadrants in a cartesian graph
    private double[][] wheelAngleSpeed = new double[4][2]; // angle, speed format

    // calculates angle and speed for all swerve modules provided a translation vector
    // broken up into x (strafePower) and y (forward) power components
    // and a rotation vector
    public void calculateKinematics(double rotationPower, double strafePower, double forwardPower) {
        // math functions used to calculate angle and speed
        double vectorMath1 = strafePower - (rotationPower * wheelbase);
        double vectorMath2 = strafePower + (rotationPower * wheelbase);
        double vectorMath3 = forwardPower - (rotationPower * trackwidth);
        double vectorMath4 = forwardPower + (rotationPower * trackwidth);

        wheelAngleSpeed[0][0] = Math.atan2(vectorMath2, vectorMath3); //RF angle
        wheelAngleSpeed[0][1] = Math.sqrt(Math.pow(vectorMath2, 2) + Math.pow(vectorMath3, 2)); //RF speed

        wheelAngleSpeed[1][0] = Math.atan2(vectorMath2, vectorMath4); //LF angle
        wheelAngleSpeed[1][1] = Math.sqrt(Math.pow(vectorMath2, 2) + Math.pow(vectorMath4, 2)); //LF speed

        wheelAngleSpeed[2][0] = Math.atan2(vectorMath1, vectorMath4); //LB angle
        wheelAngleSpeed[2][1] = Math.sqrt(Math.pow(vectorMath1, 2) + Math.pow(vectorMath4, 2)); //LB speed

        wheelAngleSpeed[3][0] = Math.atan2(vectorMath1, vectorMath3); //RB angle
        wheelAngleSpeed[3][1] = Math.sqrt(Math.pow(vectorMath1, 2) + Math.pow(vectorMath3, 2)); //RB speed

        // normalizes powers if the requested power is above 1 or below -1
        normalizePowers();
    }

    // parses wheelAngleSpeed 2D array and returns 1D array of just angles
    public double[] getWheelAngles() {
        double[] wheelAngles = new double[4];
        for (int i = 0; i < 4; i++)
            wheelAngles[i] = wheelAngleSpeed[i][0];
        return wheelAngles;
    }

    // parses wheelAngleSpeed 2D array and returns 1D array of drive powers
    public double[] getWheelVelocities() {
        double[] wheelVelocities = new double[4];
        for (int i = 0; i < 4; i++)
            wheelVelocities[i] = wheelAngleSpeed[i][1];
        return wheelVelocities;
    }

    // function to normalize drive powers if the requested power is above 1 or below -1
    public void normalizePowers() {
        double max = 0;
        for(int i = 0; i < 4; i++) {
            if(Math.abs(wheelAngleSpeed[i][1]) > max)
                max = Math.abs(wheelAngleSpeed[i][1]);
        }
        if(max > 1.0) {
            for(int i = 0; i < 4; i++) {
                wheelAngleSpeed[i][1] /= max;
            }
        }
    }

    // functions to set wheelbase and trackwidth measurements
    public void setWheelbase(double wheelbase) { this.wheelbase = wheelbase; }
    public void setTrackwidth(double trackwidth) { this.trackwidth = trackwidth; }
}
