package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "SampleTestOpMode", group = "TestOpModes")
public class SampleTestOpMode extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap); // creates new robot


        // teleop managers


        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            robot.update();
        }
    }
}
