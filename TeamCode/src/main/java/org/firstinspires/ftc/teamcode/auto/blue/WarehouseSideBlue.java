package org.firstinspires.ftc.teamcode.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.RemoteAuto;
import org.firstinspires.ftc.teamcode.auto.autocontrol.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.autocontrol.RunToPosition;
import org.firstinspires.ftc.teamcode.swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;

@Autonomous(name = "Blue Warehouse Side", group = "Auto")
public class WarehouseSideBlue extends LinearOpMode {
    Robot robot;
    RunToPosition drive;
    Vision vision;

    ElapsedTime currentTime;
    double time;

    public enum LEVEL {
        HIGH,
        MID,
        LOW
    }
    RemoteAuto.LEVEL level;

    int cycles = 2; // number of extra cycles

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, gamepad1, gamepad2);
        drive = new RunToPosition(
                hardwareMap,
                DriveConstants.drivePIDConstants,
                DriveConstants.rotationPIDconstants);
        drive.setStartPose(12, 64, Math.toRadians(90));
        drive.setTargetHeading(Math.toRadians(90));

        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();
        currentTime = new ElapsedTime();

        // run vision and update enum
        while(!opModeIsActive() && !isStopRequested()) {
            VisionPipeline.POS pos = vision.getPosition();
            if(pos == VisionPipeline.POS.RIGHT)
                level = RemoteAuto.LEVEL.HIGH;
            else if(pos == VisionPipeline.POS.CENTER)
                level = RemoteAuto.LEVEL.MID;
            else if(pos == VisionPipeline.POS.LEFT)
                level = RemoteAuto.LEVEL.LOW;
        }
        if(isStopRequested()) return;
        waitForStart();
        vision.closeCamera();
        currentTime.reset();
        drive.resetTimers();

        // AUTO CODE
        robot.claw.grip();
        robot.lift.rest();
        robot.arm.intake();
        robot.update();

        // SCORE PRELOAD
        timeout(0.25);

        // RANDOMIZATION SWITCH CASE
        switch (level) {
            case LOW:
                robot.arm.low();
                robot.lift.rest();
                drive.setTargetPoint(-1, 39);
                break;
            case MID:
                robot.arm.mid();
                robot.lift.mid();
                drive.setTargetPoint(-1, 39);
                break;
            case HIGH:
                robot.arm.high();
                robot.lift.high();
                drive.setTargetPoint(-2, 36);
                break;
        }

        drive.setTargetHeading(Math.toRadians(130));
        runDrive();
        timeout(0.5);
        robot.claw.deposit();
        timeout(0.75);

        // RESET
        drive.setTargetPoint(2, 45);
        runDrive();
        robot.claw.timedRetract();
        robot.arm.intake();
        robot.lift.rest();
        timeout(0.5);

        // CYCLE LOOP
        for(int i = 0; i < cycles; i++) {

            // WAREHOUSE INTAKE CYCLE
            DriveConstants.admissibleHeadingError = Math.toRadians(5);
            DriveConstants.admissibleError = 1.65;
            drive.setTargetHeading(Math.toRadians(180));
            drive.setTargetPoint(8, 58);
            runDrive();
            drive.setTargetPoint(8, 62);
            runDrive();
            robot.intake.on();
            drive.setTargetPoint(40, 62);
            runDrive();
            drive.forwardByTime(0.25, 0.95);
            runDrive();
            robot.intake.reverse();
            robot.claw.grip();

            // DRIVE TO SCORE CARGO
            DriveConstants.admissibleError = 2;
            drive.setTargetPoint(12, 62);
            runDrive();
            robot.intake.off();
            timeout(0.5);
            robot.intake.reverse();
            DriveConstants.admissibleHeadingError = Math.toRadians(10);
            drive.setTargetHeading(Math.toRadians(125));
            drive.setTargetPoint(-2, 36);
            runDrive();

            // DEPOSIT CARGO
            robot.intake.off();
            robot.lift.high();
            robot.arm.high();
            timeout(0.75);
            robot.claw.deposit();
            timeout(0.5);

            // RESET
            robot.claw.timedRetract();
            robot.arm.intake();
            robot.lift.rest();
        }

        // PARK
        drive.setTargetHeading(Math.toRadians(180));
        drive.setTargetPoint(16, 62);
        runDrive();
        drive.setTargetPoint(45, 62);
        runDrive();
    }

    public void runDrive() {
        drive.update();
        while(!drive.isReachedTarget() && opModeIsActive()) {
            drive.update();
            robot.update();
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