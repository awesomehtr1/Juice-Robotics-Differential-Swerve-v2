package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2); // creates new robot

        // teleop managers
        // DRIVER 1
        robot.createTeleOpManager(new TeleOpManagerBuilder() // intake
                .typeToggle(()-> gamepad1.right_trigger > 0.1)
                .addAction(()-> robot.intake.on())
                .addAction(()-> robot.intake.off())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // slowmode
                .typeToggle(()-> gamepad1.left_trigger > 0.1)
                .addAction(()-> robot.drive.setSlowmode(true))
                .addAction(()-> robot.drive.setSlowmode(false))
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // scoring high
                .typeTrigger(()-> gamepad1.dpad_up)
                .addAction(()-> robot.claw.grip())
                .addAction(()-> robot.lift.high())
                .addAction(()-> robot.arm.depositHigh())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // scoring mid
                .typeTrigger(()-> gamepad1.dpad_left)
                .addAction(()-> robot.claw.grip())
                .addAction(()-> robot.lift.mid())
                .addAction(()-> robot.arm.depositMid())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // scoring low
                .typeTrigger(()-> gamepad1.dpad_right)
                .addAction(()-> robot.claw.grip())
                .addAction(()-> robot.lift.rest())
                .addAction(()-> robot.arm.depositLow())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // release cargo
                .typeTrigger(()-> gamepad1.a)
                .addAction(()-> robot.claw.intake())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // reset for intaking
                .typeTrigger(()-> gamepad1.dpad_down)
                .addAction(()-> robot.lift.rest())
                .addAction(()-> robot.arm.intake())
                .addAction(()-> robot.claw.intake())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // duck spinner
                .typeToggle(()-> gamepad1.x)
                .addAction(()-> robot.spinner.on())
                .addAction(()-> robot.spinner.off())
                .build());

        // reset robot
        robot.lift.rest();
        robot.arm.intake();
        robot.claw.intake();
        robot.update();

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            robot.drive.gamepadInput(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );
            robot.update();
        }
    }
}