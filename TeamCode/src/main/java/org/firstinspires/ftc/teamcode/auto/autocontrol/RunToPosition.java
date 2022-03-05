package org.firstinspires.ftc.teamcode.auto.autocontrol;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;
import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class RunToPosition {
    private SwerveDrive swerveDrive;
    private PDVAController pdvaController;
    private MotionProfile motionProfile;
    private BasicPID rotationPID;
    private ElapsedTime time;
    private SanfordGyro gyro;
    private ElapsedTime odoTime;

    private double targetX;
    private double targetHeading;
    private double targetY;

    private double prevTime;

    public RunToPosition(
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

    public void update() {
        // update pose
        updatePose();

        // update motion profile
        double distanceToTarget = distanceBetweenPoints(
                swerveDrive.getX(),
                swerveDrive.getY(),
                targetX,
                targetY
        );
        motionProfile.update(
                swerveDrive.getVelocity(),
                distanceToTarget,
                0,
                odoTime.milliseconds() - prevTime
        );

        // update rotation PID
        rotationPID.setState(targetHeading);
        rotationPID.updatePID(targetHeading);

        // set drivetrain power
        double rotation = rotationPID.getPower();
        double translationPower = pdvaController.getOutput();
        double feedforwardPower = pdvaController.getFeedForwardOutput();
        double angle = vectorAngle(
                swerveDrive.getX(),
                swerveDrive.getY(),
                targetX,
                targetY
        );
        double strafe, forward;
        if(converge(distanceToTarget)) {
            strafe = Math.cos(angle) * translationPower;
            forward = Math.sin(angle) * translationPower;
        }
        else {
            strafe = Math.cos(angle) * feedforwardPower;
            forward = Math.sin(angle) * feedforwardPower;
        }
        swerveDrive.setMotorPowers(rotation, strafe, forward);
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

    public boolean converge(double distanceToTarget) {
        return distanceToTarget < DriveConstants.convergeDistance;
    }

    public void setTargetPoint(double x, double y) {
        targetX = x;
        targetY = y;
    }

    public void setTargetHeading(double heading) {
        targetHeading = heading;
    }

    public void setStartPose(double x, double y, double heading) {
        swerveDrive.setPose(x, y, heading);
        targetX = x;
        targetY = y;
        targetHeading = heading;
    }

    public boolean reachedTarget(double distanceToTarget) {
        return distanceToTarget < DriveConstants.admissibleError;
    }
}
