package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.teleopmanager.RobotAction;
import org.firstinspires.ftc.teamcode.teleopmanager.TeleOpManager;
import org.firstinspires.ftc.teamcode.teleopmanager.TeleOpManagerBuilder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap); // creates new robot

        // teleop managers
        // DRIVER 1
        robot.createTeleOpManager(new TeleOpManagerBuilder()
                .typeToggle(()-> gamepad1.right_trigger > 0.05)
                .addAction(()-> robot.intake.on())
                .addAction(()-> robot.intake.off())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder()
                .typeTrigger(()-> gamepad1.dpad_up)
                .addAction(()-> robot.claw.grip())
                .addAction(()-> robot.lift.high())
                .addAction(()-> robot.arm.deposit())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder()
                .typeTrigger(()-> gamepad1.dpad_left)
                .addAction(()-> robot.claw.grip())
                .addAction(()-> robot.lift.mid())
                .addAction(()-> robot.arm.deposit())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder()
                .typeTrigger(()-> gamepad1.dpad_right)
                .addAction(()-> robot.claw.grip())
                .addAction(()-> robot.lift.low())
                .addAction(()-> robot.arm.deposit())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder()
                .typeTrigger(()-> gamepad1.dpad_down)
                .addAction(()-> robot.lift.rest())
                .addAction(()-> robot.arm.intake())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder()
                .typeTrigger(()-> gamepad1.x)
                .addAction(()-> robot.claw.release())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder()
                .typeTrigger(()-> gamepad1.a)
                .addAction(()-> robot.claw.grip())
                .build());

        // DRIVER 2
        robot.createTeleOpManager(new TeleOpManagerBuilder()
                .typeToggle(()-> gamepad2.x)
                .addAction(()-> robot.spinner.on())
                .addAction(()-> robot.spinner.off())
                .build()
        );

        // reset robot
        robot.lift.rest();
        robot.arm.intake();
        robot.claw.rest();
        robot.update();

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