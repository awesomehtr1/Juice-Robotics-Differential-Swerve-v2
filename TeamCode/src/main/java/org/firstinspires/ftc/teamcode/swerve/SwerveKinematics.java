package org.firstinspires.ftc.teamcode.swerve;

public class SwerveKinematics {
    private double trackwidth, wheelbase;

    private double[][] wheelAngleSpeed = new double[4][2]; // angle, speed format

    public void calculateKinematics(double rotationPower, double strafePower, double forwardPower) {
        double vectorMath1 = strafePower - (rotationPower * wheelbase);
        double vectorMath2 = strafePower + (rotationPower * wheelbase);
        double vectorMath3 = forwardPower - (rotationPower * trackwidth);
        double vectorMath4 = forwardPower + (rotationPower * trackwidth);

        wheelAngleSpeed[0][0] = Math.atan2(vectorMath2, vectorMath3); //LF angle
        wheelAngleSpeed[0][1] = Math.sqrt(Math.pow(vectorMath2, 2) + Math.pow(vectorMath3, 2)); //LF speed

        wheelAngleSpeed[1][0] = Math.atan2(vectorMath2, vectorMath4); //RF angle
        wheelAngleSpeed[1][1] = Math.sqrt(Math.pow(vectorMath2, 2) + Math.pow(vectorMath4, 2)); //RF speed

        wheelAngleSpeed[2][0] = Math.atan2(vectorMath1, vectorMath4); //RB angle
        wheelAngleSpeed[2][1] = Math.sqrt(Math.pow(vectorMath1, 2) + Math.pow(vectorMath4, 2)); //RB speed

        wheelAngleSpeed[3][0] = Math.atan2(vectorMath1, vectorMath3); //LB angle
        wheelAngleSpeed[3][1] = Math.sqrt(Math.pow(vectorMath1, 2) + Math.pow(vectorMath3, 2)); //LB speed

        normalizePowers();
    }

    public double[] getWheelAngles() {
        double[] wheelAngles = new double[4];
        for (int i = 0; i < 4; i++)
            wheelAngles[i] = wheelAngleSpeed[i][0];
        return wheelAngles;
    }

    public double[] getWheelVelocities() {
        double[] wheelVelocities = new double[4];
        for (int i = 0; i < 4; i++)
            wheelVelocities[i] = wheelAngleSpeed[i][1];
        return wheelVelocities;
    }

    public void normalizePowers() {
        double max = 0;
        for(int i = 0; i < 4; i++) {
            if(wheelAngleSpeed[i][1] > max)
                max = wheelAngleSpeed[i][1];
        }
        if(max > 1.0) {
            for(int i = 0; i < 4; i++) {
                wheelAngleSpeed[i][1] /= max;
            }
        }
    }

    public void setWheelbase(double wheelbase) { this.wheelbase = wheelbase; }
    public void setTrackwidth(double trackwidth) { this.trackwidth = trackwidth; }
}
