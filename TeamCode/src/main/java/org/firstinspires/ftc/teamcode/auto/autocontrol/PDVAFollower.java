package org.firstinspires.ftc.teamcode.auto.autocontrol;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;
import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class PDVAFollower {
    private SwerveDrive swerveDrive;
    private PDVAController pdvaController;
    private MotionProfile motionProfile;
    private BasicPID rotationPID;
    private ElapsedTime time;
    private SanfordGyro gyro;
    private ElapsedTime odoTime;

    private Path path;

    private double lookaheadRadius;

    private double prevTime;

    private double targetHeading;

    public PDVAFollower(
            HardwareMap hardwareMap,
            double[] PDVAconstants,
            double[] motionProfileConstants,
            double[] rotationPIDconstants)
    {
        time = new ElapsedTime();
        odoTime = new ElapsedTime();

        swerveDrive = new SwerveDrive(hardwareMap);
        pdvaController = new PDVAController(
                PDVAconstants[0],
                PDVAconstants[1],
                PDVAconstants[2],
                PDVAconstants[3]
        );
        motionProfile = new MotionProfile(
                motionProfileConstants[0],
                motionProfileConstants[1],
                motionProfileConstants[2],
                motionProfileConstants[3]
        );
        rotationPID = new BasicPID(
                rotationPIDconstants[0],
                rotationPIDconstants[1],
                rotationPIDconstants[2],
                rotationPIDconstants[3],
                time);
        gyro = new SanfordGyro(hardwareMap);
    }

    public void updateFollower() {
        // update pose
        updatePose();

        // update motion profile
        double distanceToTarget = distanceBetweenPoints(
                swerveDrive.getX(),
                swerveDrive.getY(),
                path.getSegmentEnd().x,
                path.getSegmentEnd().y
        );
        motionProfile.update(
                swerveDrive.getVelocity(),
                distanceToTarget,
                path.getEndVelo(),
                odoTime.milliseconds() - prevTime
        );

        // update rotation PID
        rotationPID.setState(targetHeading);
        rotationPID.updatePID(targetHeading);

        // update PDVA controller
        double target[] = getTargetPoint();
        double PDVAerror = distanceBetweenPoints(
                swerveDrive.getX(),
                swerveDrive.getY(),
                target[0],
                target[1]
        );
        pdvaController.setTarget(PDVAerror);
        pdvaController.update(
                0,
                motionProfile.getTargetVelo(),
                motionProfile.getTargetAccel()
        );

        // set drivetrain power
        double rotation = rotationPID.getPower();
        double translationPower = pdvaController.getOutput();
        double angle = vectorAngle(
                swerveDrive.getX(),
                swerveDrive.getY(),
                target[0],
                target[1]
        );
        double strafe = Math.cos(angle) * translationPower;
        double forward = Math.sin(angle) * translationPower;
        swerveDrive.setMotorPowers(rotation, strafe, forward);
    }

    public double[] getTargetPoint() {
        // calculate linear equation constants for path segment
        double rise = path.getSegmentEnd().y - path.getSegmentStart().y;
        double run = path.getSegmentEnd().x - path.getSegmentEnd().x;
        double m, b;

        // circle around robot
        double x = swerveDrive.getX();
        double y = swerveDrive.getY();
        double r = lookaheadRadius;

        double solution1, solution2;

        double[] solution = new double[2];

        int invalid = 99999;

        // check for edge case with vertical line
        if(run != 0) {
            double root = Math.sqrt((r*r)-Math.pow((path.getSegmentStart().x-x),2));
            solution1 = y+root;
            solution2 = y-root;

            double min = Math.min(path.getSegmentStart().y, path.getSegmentEnd().y);
            double max = Math.max(path.getSegmentStart().y, path.getSegmentEnd().y);

            // check for solutions out of bounds
            if(solution1 < min || solution1 > max)
                solution1 = invalid;
            if(solution2 < min || solution2 > max)
                solution1 = invalid;

            // if both solutions are valid, pick the one closest to the end point
            double distance1 = distanceBetweenPoints(
                    path.getSegmentEnd().x,
                    solution1,
                    path.getSegmentEnd().x,
                    path.getSegmentEnd().y
            );
            double distance2 = distanceBetweenPoints(
                    path.getSegmentEnd().x,
                    solution2,
                    path.getSegmentEnd().x,
                    path.getSegmentEnd().y
            );
            if(distance1 < distance2) {
                solution[0] = path.getSegmentEnd().x;
                solution[1] = solution1;
            }
            if(distance1 > distance2) {
                solution[0] = path.getSegmentEnd().x;
                solution[1] = solution2;
            }

            // check for edge cases (eg path shorter than circle radius; solution is farther from end than robot)
            if(solution1 == invalid && solution2 == invalid) {
                solution[0] = path.getSegmentEnd().x;
                solution[1] = path.getSegmentEnd().y;
            }
            double solutionToEnd = distanceBetweenPoints(
                    path.getSegmentEnd().x,
                    solution[1],
                    path.getSegmentEnd().x,
                    path.getSegmentEnd().y
            );
            double robotToEnd = distanceBetweenPoints(
                    x,
                    y,
                    path.getSegmentEnd().x,
                    path.getSegmentEnd().y
            );
            if(solutionToEnd > robotToEnd) {
                solution[0] = path.getSegmentEnd().x;
                solution[1] = path.getSegmentEnd().y;
            }
            return solution;
        }

        else {
            m = rise / run;
            b = path.getSegmentStart().y - path.getSegmentStart().x * m;

            // solution exists
            try {
                // equation for finding intersection of circle and line
                double outsideRoot = (m*y)-(m*b)+r;
                double insideRoot = -(y*y)+(2*b*y)+(2*m*y*x)+(m*m*r*r)+(r*r)-(m*m*x*x)-(2*m*b*x)-(b*b);
                double root = Math.sqrt(insideRoot);
                double denominator = (m*m)+1;
                solution1 = (outsideRoot + root) / denominator;
                solution2 = (outsideRoot - root) / denominator;
            }

            // no solution exists (circle does not intersect line)
            catch (Exception e) {
                // get intersection with perpendicular line
                double numerator = (m*y)+x-(m*b);
                double denominator = (m*m)+1;
                double solutionX = numerator / denominator;
                double solutionY = m * solutionX + b;

                // check if solution is within path segment
                double min = Math.min(path.getSegmentStart().x, path.getSegmentEnd().x);
                double max = Math.max(path.getSegmentStart().x, path.getSegmentEnd().x);

                // check for solutions out of bounds
                if(solutionX < min || solutionX > max) {
                    solutionX = path.getSegmentEnd().x;
                    solutionY = path.getSegmentEnd().y;
                }

                double solutionToEnd = distanceBetweenPoints(
                        solutionX,
                        solutionY,
                        path.getSegmentEnd().x,
                        path.getSegmentEnd().y
                );
                double robotToEnd = distanceBetweenPoints(
                        x,
                        y,
                        path.getSegmentEnd().x,
                        path.getSegmentEnd().y
                );
                if(solutionToEnd > robotToEnd) {
                    solutionX = path.getSegmentEnd().x;
                    solutionY = path.getSegmentEnd().y;
                }

                solution[0] = solutionX;
                solution[1] = solutionY;
                return solution;
            }
        }

        double solutionY;

        // check if solutions are within bounds; invalid if outside bounds
        double min = Math.min(path.getSegmentStart().x, path.getSegmentEnd().x);
        double max = Math.max(path.getSegmentStart().x, path.getSegmentEnd().x);

        // check for solutions out of bounds
        if(solution1 < min || solution1 > max)
            solution1 = invalid;
        if(solution2 < min || solution2 > max)
            solution1 = invalid;

        // if solution == invalid, set valid solution
        if(solution1 == invalid) {
            solutionY = m *  solution2 + b;
            solution[0] = solution2;
            solution[1] = solutionY;
        }
        if(solution2 == invalid) {
            solutionY = m *  solution1 + b;
            solution[0] = solution1;
            solution[1] = solutionY;
        }

        // if both solutions are valid, pick the one closest to the end point
        double distance1 = distanceBetweenPoints(
                solution1,
                (m *  solution1 + b),
                path.getSegmentEnd().x,
                path.getSegmentEnd().y
        );
        double distance2 = distanceBetweenPoints(
                solution2,
                (m *  solution2 + b),
                path.getSegmentEnd().x,
                path.getSegmentEnd().y
        );
        if(distance1 < distance2) {
            solutionY = m *  solution1 + b;
            solution[0] = solution1;
            solution[1] = solutionY;
        }
        if(distance1 > distance2) {
            solutionY = m *  solution2 + b;
            solution[0] = solution2;
            solution[1] = solutionY;
        }

        // check for edge cases (eg path shorter than circle radius; solution is farther from end than robot)
        if(solution1 == invalid && solution2 == invalid) {
            solution[0] = path.getSegmentEnd().x;
            solution[1] = path.getSegmentEnd().y;
        }
        double solutionToEnd = distanceBetweenPoints(
                solution[0],
                solution[1],
                path.getSegmentEnd().x,
                path.getSegmentEnd().y
        );
        double robotToEnd = distanceBetweenPoints(
                x,
                y,
                path.getSegmentEnd().x,
                path.getSegmentEnd().y
        );
        if(solutionToEnd > robotToEnd) {
            solution[0] = path.getSegmentEnd().x;
            solution[1] = path.getSegmentEnd().y;
        }

        return solution;
    }

    public double distanceBetweenPoints(double x1, double y1, double x2, double y2) {
        double xDiff = x2 - x1;
        double yDiff = y2 - y1;
        return Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff, 2));
    }

    public double vectorAngle(double x1, double y1, double x2, double y2) {
        double x = x2 - x1;
        double y = y2 - y1;
        return Math.atan(y / x);
    }

    public void updatePose() {
        swerveDrive.updatePose(gyro.getLowPassEstimate(), odoTime.milliseconds() - prevTime);
        prevTime = odoTime.milliseconds();
    }

    public void setTargetHeading(double heading) {
        targetHeading = heading;
    }

    public void setLookaheadRadius(double lookaheadRadius) {
        this.lookaheadRadius = lookaheadRadius;
    }

    public void setStartPose(double x, double y, double heading) {
        swerveDrive.setPose(x, y, heading);
    }

    public void setPath(Path path) {
        this.path = path;
    }
}