package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.autocontrol.TimeBasedDrive;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;

@Autonomous(name = "Remote Auto", group = "Auto")
public class RemoteAuto extends LinearOpMode {
    Robot robot;
    TimeBasedDrive drive;
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
        drive = new TimeBasedDrive(robot, hardwareMap, telemetry);
        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();

        currentTime = new ElapsedTime();

        // run vision and update enum
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

        drive.rotateTo(0);

        // move away from wall
        drive.forward(false, 0.25);
        runDrive();

        // strafe to score preload
        drive.strafe(true, 1.56);
        runDrive();

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

        // driving backwards to score preload
        drive.forward(false, 1.28);
        runDrive();

        timeout(0.25);
        robot.claw.deposit();
        timeout(0.25);

        // get out of the way of the hub
        drive.forward(true, 0.4);
        runDrive();

        // reset
        robot.claw.timedRetract();
        robot.arm.intake();
        robot.lift.rest();
        timeout(0.5);

        // driving to intake duck
        robot.intake.on();
        drive.rotateTo(Math.toRadians(104));
        timeout(0.5);
        timeout(1.0);
        drive.setPower(0.4);
        drive.forward(true,1.1);
        runDrive();
        drive.rotateTo(Math.toRadians(90));
        timeout(0.5);
        drive.setPower(0.25);
        drive.forward(true, 0.75);
        runDrive();

        // scoring duck
        robot.intake.off();
        drive.setPower(0.3);
        drive.strafe(true, 1);
        runDrive();

        robot.claw.grip();
        timeout(0.5);
        robot.arm.high();
        robot.lift.high();
        timeout(0.5);
        drive.forward(false, 0.65);
        runDrive();

        timeout(0.25);
        robot.claw.deposit();
        timeout(0.25);

        // reset
        robot.claw.timedRetract();
        robot.arm.intake();
        robot.lift.rest();

        // warehouse intake cycle
        drive.setStrafePower(0.5);
        drive.strafe(false, 2.25);
        runDrive();
        drive.setStrafePower(0.3);
        robot.intake.on();
        drive.setPower(0.55);
        drive.forward(true,1.5);
        runDrive();
        timeout(0.5);

        // move to cycle cargo
        robot.intake.reverse();
        robot.claw.grip();
        drive.forward(false, 0.65);
        runDrive();
        drive.strafe(false, 0.6);
        runDrive();
        drive.forward(false, 0.85);
        runDrive();
        drive.strafe(true, 2.2);
        runDrive();

        // scoring cargo
        robot.intake.off();
        robot.arm.high();
        robot.lift.high();
        timeout(1.0);
        robot.claw.deposit();
        timeout(0.5);

        // reset
        robot.claw.timedRetract();
        robot.arm.intake();
        robot.lift.rest();

        // park
        drive.setStrafePower(0.6);
        drive.strafe(false, 2);
        runDrive();
        drive.forward(true,1.2);
        runDrive();
    }

    public void runDrive() {
        while(drive.isMoving() && opModeIsActive()) {
            drive.update();
            robot.update();
        }
        drive.stopDrive();
    }

    public void timeout(double seconds) {
        currentTime.reset();
        this.time = seconds;
        while(isTimedOut() && opModeIsActive()){
            robot.update();
            drive.update();
        }
        drive.turnOnCorrection();
    }

    public boolean isTimedOut() { return currentTime.seconds() < time; }
}
