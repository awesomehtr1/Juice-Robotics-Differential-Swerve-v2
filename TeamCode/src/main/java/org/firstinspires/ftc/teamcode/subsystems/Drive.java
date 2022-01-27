package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveRR;

public class Drive implements Subsystem {
    Robot robot;
    public SwerveDriveRR drive;
    public static boolean SanfordGyro = false;
    boolean teleopDriveOn = false;
    double drivePower = 1.0;

    public Drive(Robot robot){
        this.robot = robot;
        drive = new SwerveDriveRR(robot.hardwareMap, SanfordGyro);
    }

    public void slowModeOn() {drivePower = 0.3;}
    public void slowModeOff() {drivePower = 1.0;}

    public void teleopDriveOn() {teleopDriveOn = true;}

    @Override
    public void update() {
        if(teleopDriveOn) {
            drive.setWeightedDrivePower(new Pose2d(
                    robot.gamepad1.left_stick_y * drivePower,
                    robot.gamepad1.left_stick_x * drivePower,
                    robot.gamepad1.right_stick_x * drivePower));
        }
    }
}