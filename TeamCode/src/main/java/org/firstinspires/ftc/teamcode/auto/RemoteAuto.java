package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.autocontrol.Drive;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;

@Autonomous(name = "Remote Auto", group = "Auto")
public class RemoteAuto extends LinearOpMode {
    Robot robot;
    Drive drive;
    Vision vision;

    ElapsedTime currentTime;
    double time;

    public enum LEVEL {
        HIGH,
        MID,
        LOW
    }
    LEVEL level;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, gamepad1, gamepad2);
        drive = new Drive(robot, hardwareMap);
        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();

        currentTime = new ElapsedTime();

        while(!opModeIsActive() && !isStopRequested()) {
            VisionPipeline.POS pos = vision.getPosition();
            if(pos == VisionPipeline.POS.RIGHT)
                level = LEVEL.HIGH;
            else if(pos == VisionPipeline.POS.CENTER)
                level = LEVEL.MID;
            else if(pos == VisionPipeline.POS.LEFT)
                level = LEVEL.LOW;
        }
        if(isStopRequested()) return;
        waitForStart();
        vision.closeCamera();

        // AUTO CODE
        robot.claw.grip();
        robot.lift.rest();
        robot.arm.intake();
        robot.update();

        // scoring duck
        drive.stopRotation();
        timeout(1.25);
        drive.startRotation();
        drive.rotateTo(Math.toRadians(-90));
        timeout(2);
        drive.forward(true, 1.5);
        runDrive();
        drive.stopDrive();
        robot.spinner.on();
        robot.update();
        robot.spinner.off();
        robot.update();

        // scoring preload
        drive.rotateTo(Math.toRadians(30));
        timeout(2);
        drive.forward(false, 3);
        runDrive();
        drive.stopDrive();

        // moving lift/arm to correct level
        if(level == LEVEL.LOW)
            robot.arm.low();
        else if(level == LEVEL.MID) {
            robot.arm.mid();
            robot.lift.mid();
        }
        else if(level == LEVEL.HIGH) {
            robot.arm.high();
            robot.lift.high();
        }
        robot.update();

        timeout(1);
        robot.claw.intake();
        robot.update();

        // parking
        drive.strafe(true, 1);
        runDrive();
        robot.arm.intake();
        robot.lift.rest();
        robot.update();
        drive.rotateTo(Math.toRadians(90));
        timeout(2);
        drive.strafe(true, 2);
        runDrive();
        drive.stopDrive();
        drive.forward(false, 4);
        runDrive();
        drive.stopDrive();
    }

    public void runDrive() {
        while(drive.isMoving() && opModeIsActive()) {
            drive.update();
        }
    }

    public void timeout(double seconds) {
        currentTime.reset();
        this.time = seconds;
        while(isTimedOut() && opModeIsActive()){
            robot.update();
            drive.update();
        }
    }

    public boolean isTimedOut() { return currentTime.seconds() < time; }
}
