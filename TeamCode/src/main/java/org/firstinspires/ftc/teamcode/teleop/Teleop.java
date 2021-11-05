package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2); // creates new robot

        // teleop managers


        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            robot.drive.drive.setWeightedDrivePower(new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x));
            robot.update();
        }
    }
}
