package org.firstinspires.ftc.teamcode.auto.autocontrol;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;
import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class Drive {
    private SwerveDrive swerveDrive;
    private SanfordGyro gyro;

    private Robot robot;

    private ElapsedTime currentTime, PIDtime;
    private double time;

    private double rotation, strafe, forward;
    private double angle;

    private BasicPID rotationPID;

    private double power;
    private double rotPower;

    private boolean moving, rotate;

    private VoltageSensor voltageSensor;

    public Drive(Robot robot, HardwareMap hardwareMap) {
        swerveDrive = new SwerveDrive(hardwareMap);
        swerveDrive.setSlowmode(false);
        this.robot = robot;
        gyro = new SanfordGyro(hardwareMap);
        currentTime = new ElapsedTime();
        time = 0.0;
        PIDtime = new ElapsedTime();
        rotationPID = new BasicPID(1.2, 0.25, 0.245, 0, PIDtime);
        rotationPID.setState(0);
        power = 0.6;
        rotPower = 0.8;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void update() {
        rotation = rotationPID.updatePID(getAngle()) * rotPower;
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
                rotation * voltageCompensation * 2,
                strafe * voltageCompensation,
                forward * voltageCompensation);
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
        strafe = right ? power : -power;
        moving = true;
    }

    public void rotateTo(double angle) {
        this.angle = angle;
        rotationPID.setState(angle);
    }

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
}
