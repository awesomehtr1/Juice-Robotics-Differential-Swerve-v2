package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "SampleTestOpMode", group = "TestOpModes")
public class SampleTestOpMode extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2); // creates new robot

        // teleop managers


        waitForStart();
        while(opModeIsActive()){
            robot.update();
        }
    }
}
