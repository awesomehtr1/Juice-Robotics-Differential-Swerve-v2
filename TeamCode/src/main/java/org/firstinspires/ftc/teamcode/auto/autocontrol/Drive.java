package org.firstinspires.ftc.teamcode.auto.autocontrol;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;
import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class Drive {
    private SwerveDrive swerveDrive;
    private SanfordGyro gyro;

    private Robot robot;

    private ElapsedTime currentTime, PIDtime, correctionPIDtime;
    private double time;

    private double rotation, strafe, forward;
    private double angle;

    private BasicPID rotationPID, correctionPID;

    private double power, rotPower, strafePower;

    private boolean moving, rotate, correction;

    private VoltageSensor voltageSensor;
    Telemetry telemetry;

    public Drive(Robot robot, HardwareMap hardwareMap, Telemetry telemetry) {
        swerveDrive = new SwerveDrive(hardwareMap);
        swerveDrive.setSlowmode(false);
        swerveDrive.setBrake();
        this.robot = robot;
        gyro = new SanfordGyro(hardwareMap);
        currentTime = new ElapsedTime();
        time = 0.0;
        PIDtime = new ElapsedTime();
        correctionPIDtime = new ElapsedTime();
        rotationPID = new BasicPID(0.72, 0.06, 0.08, 0.0, PIDtime);
        correctionPID = new BasicPID(1.8, 0.0, 0.17,0, correctionPIDtime);
        rotationPID.setState(0);
        power = 0.3;
        strafePower = 0.3;
        rotPower = 1.0;
        rotate = true;
        correction = true;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.telemetry = telemetry;
    }

    public void update() {
        double gyroAngle = getAngle();
        if(correction)
            rotation = correctionPID.updatePID(gyroAngle) * rotPower;
        else
            rotation = rotationPID.updatePID(gyroAngle) * rotPower;
        if(currentTime.seconds() >= time) {
            forward = 0;
            strafe = 0;
        }
        if(forward != 0 || strafe != 0)
            moving = true;
        else
            moving = false;

        double voltageCompensation = 13.6 / voltageSensor.getVoltage();
        if(!rotate)
            rotation = 0;
        swerveDrive.setMotorPowers(
                rotation,
                strafe * voltageCompensation,
                forward * voltageCompensation);

        telemetry.addData("voltage: ", voltageSensor.getVoltage());
    }

    public void forward(boolean forward, double time) {
        currentTime.reset();
        this.time = time;
        this.forward = forward ? power : -power;
        moving = true;
    }

    public void strafe(boolean right, double time) {
        currentTime.reset();
        this.time = time;
        strafe = right ? strafePower : -strafePower;
        moving = true;
    }

    public void rotateTo(double angle) {
        correction = false;
        this.angle = angle;
        rotationPID.setState(angle);
        correctionPID.setState(angle);
    }

    public void setPower(double power) { this.power = power; }

    public void setStrafePower(double power) { strafePower = power; }

    public double getAngle() { return gyro.getLowPassEstimate(); }

    public boolean isMoving() { return moving; }

    public void stopDrive() {
        moving = false;
        forward = 0;
        strafe = 0;
        update();
    }

    public void stopRotation() { rotate = false; }

    public void startRotation() { rotate = true; }

    public void turnOnCorrection() { correction = true; }
}
