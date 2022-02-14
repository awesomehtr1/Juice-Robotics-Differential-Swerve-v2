package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled
public class TeleopAlternative extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, gamepad1, gamepad2);

        robot.lift.rest();
        robot.arm.intake();
        robot.claw.intake();
        robot.update();

        waitForStart();
        while (opModeIsActive()) {
            robot.drive.gamepadInput(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            if(gamepad1.right_trigger > 0.1)
                robot.intake.on();
        }
    }
}
