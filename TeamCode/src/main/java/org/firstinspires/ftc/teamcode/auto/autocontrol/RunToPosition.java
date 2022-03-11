package org.firstinspires.ftc.teamcode.auto.autocontrol;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.MathFunctions;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;
import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class RunToPosition {
    private SwerveDrive swerveDrive;
    private BasicPID pid;
    private BasicPID rotationPID;
    private ElapsedTime time, rotationTime, odoTime, runTimer;
    private SanfordGyro gyro;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    private double targetX;
    private double targetY;
    private double targetHeading;

    private double prevTime;

    private double drivePowerCap = DriveConstants.drivePowerCap;
    private double rotationPowerCap = DriveConstants.rotationPowerCap;

    private boolean reachedTarget;
    private double timeThreshold;

    private double forwardPower, strafePower;

    public RunToPosition(
            HardwareMap hardwareMap,
            double[] drivePIDConstants,
            double[] rotationPIDconstants)
    {
        time = new ElapsedTime();
        odoTime = new ElapsedTime();
        rotationTime = new ElapsedTime();
        runTimer = new ElapsedTime();

        swerveDrive = new SwerveDrive(hardwareMap);
        swerveDrive.setSlowmode(false);
        pid = new BasicPID(
                drivePIDConstants[0],
                drivePIDConstants[1],
                drivePIDConstants[2],
                drivePIDConstants[3],
                time
        );
        rotationPID = new BasicPID(
                rotationPIDconstants[0],
                rotationPIDconstants[1],
                rotationPIDconstants[2],
                rotationPIDconstants[3],
                rotationTime
        );
        gyro = new SanfordGyro(hardwareMap);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        reachedTarget = false;
    }

    public void update() {
        // update pose
        updatePose();

        // get errors
        double distanceToTarget = distanceBetweenPoints(
                swerveDrive.getX(),
                swerveDrive.getY(),
                targetX,
                targetY
        );
        double angle = vectorAngle(
                swerveDrive.getX(),
                swerveDrive.getY(),
                targetX,
                targetY
        );
        double robotHeading = -swerveDrive.getHeading();
        double headingError = targetHeading - swerveDrive.getHeading();
        headingError = MathFunctions.angleWrap(headingError);

        // update rotation PID
        rotationPID.setState(headingError);
        rotationPID.updatePID(0);

        // update drive PID
        pid.setState(distanceToTarget);
        pid.updatePID(0);

        // set drivetrain power
        double[] rotated = rotate(Math.cos(angle), Math.sin(angle), robotHeading);
        double strafe = rotated[0];
        double forward = rotated[1];

        double rotation = rotationPID.getPower();
        double power = pid.getPower();
        strafe *= power;
        forward *= power;

        if(Math.abs(strafe) > drivePowerCap)
            strafe = Math.signum(strafe) * drivePowerCap;
        if(Math.abs(forward) > drivePowerCap)
            forward = Math.signum(forward) * drivePowerCap;
        if(Math.abs(rotation) > rotationPowerCap)
            rotation = Math.signum(rotation) * rotationPowerCap;

        if(distanceToTarget < DriveConstants.admissibleError) {
            strafe = 0;
            forward = 0;
        }
        if(Math.abs(headingError) <= DriveConstants.admissibleHeadingError) {
            rotation = 0;
        }
        reachedTarget = false;
        if(rotation == 0 && strafe == 0 && forward == 0)
            reachedTarget = true;

        // check for run by time requests
        if(forwardPower != 0 || strafePower != 0) {
            if(timeThreshold > runTimer.seconds()) {
                forward = forwardPower;
                strafe = strafePower;
                reachedTarget = false;
            }
            else {
                forwardPower = 0;
                strafePower = 0;
                reachedTarget = true;
            }
        }

        swerveDrive.setMotorPowers(rotation, strafe, forward);

        double xDash = swerveDrive.getX();
        double yDash = swerveDrive.getY();
        packet.fieldOverlay()
                .setStrokeWidth(2)
                .fillRect(xDash-6, yDash-6, 12, 12)
                .strokeLine(swerveDrive.getX(), swerveDrive.getY(), targetX, targetY);
        packet.put("x", swerveDrive.getX());
        packet.put("y", swerveDrive.getY());
        packet.put("heading", swerveDrive.getHeading());
        packet.put("heading error", headingError);
        packet.put("angle", angle);
        packet.put("strafe", rotated[1]);
        packet.put("forward", rotated[0]);
        packet.put("strafe error", Math.cos(angle) * distanceToTarget);
        packet.put("forward error", Math.sin(angle) * distanceToTarget);
        packet.put("rotation pid", rotation);
        packet.put("target heading", targetHeading);
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    public double distanceBetweenPoints(double x1, double y1, double x2, double y2) {
        double xDiff = x2 - x1;
        double yDiff = y2 - y1;
        return Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff, 2));
    }

    public double vectorAngle(double x1, double y1, double x2, double y2) {
        double x = x2 - x1;
        double y = y2 - y1;
        return Math.atan2(y, x);
    }
    public double[] rotate(double x, double y, double heading) {
        heading += Math.PI / 2;
        double xRotated = (x * Math.cos(heading)) + (y * Math.sin(heading));
        double yRotated = (-x * Math.sin(heading) + (y * Math.cos(heading)));
        double rotated[] = {xRotated, yRotated};
        return rotated;
    }

    public void updatePose() {
        swerveDrive.updatePose(gyro.getAngle(), odoTime.milliseconds() - prevTime);
        prevTime = odoTime.milliseconds();
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
        gyro.setStartingAngle(heading);
        targetHeading = heading;
    }

    public void setDrivePower(double power) { drivePowerCap = power; }

    public void setRotationPower(double power) { rotationPowerCap = power; }

    public void forwardByTime(double power, double time) {
        forwardPower = power;
        timeThreshold = time;
        runTimer.reset();
    }

    public void strafeByTime(double power, double time) {
        strafePower = power;
        timeThreshold = time;
        runTimer.reset();
    }

    public void resetTimers() {
        time.reset();
        rotationTime.reset();
        odoTime.reset();
        runTimer.reset();
    }

    public boolean isReachedTarget() { return reachedTarget; }
}
