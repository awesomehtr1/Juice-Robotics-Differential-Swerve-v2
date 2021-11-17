package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveRR;

public class Drive implements Subsystem {
    Robot robot;
    public SwerveDriveRR drive;
    public static boolean SanfordGyro = false;

    public Drive(Robot robot){
        this.robot = robot;
        drive = new SwerveDriveRR(robot.hardwareMap, SanfordGyro);
    }

    @Override
    public void update() {
    }
}