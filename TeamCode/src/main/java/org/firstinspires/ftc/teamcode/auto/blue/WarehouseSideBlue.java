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
        robot.lift.mid();
        robot.arm.intake();
        robot.update();

        // SCORE PRELOAD
        robot.arm.mid();
        drive.setTargetPoint(-2, 38);
        drive.setTargetHeading(Math.toRadians(130));
        runDrive();
        robot.claw.deposit();
        timeout(0.75);

        // RESET
        drive.setTargetPoint(2, 45);
        runDrive();
        robot.claw.timedRetract();
        robot.arm.intake();
//        robot.lift.rest();
        timeout(0.5);

        // CYCLE LOOP
        for(int i = 0; i < cycles; i++) {

            // WAREHOUSE INTAKE CYCLE
            DriveConstants.admissibleHeadingError = Math.toRadians(4);
            drive.setTargetHeading(Math.toRadians(180));
            drive.setTargetPoint(8, 58);
            runDrive();
            drive.setTargetPoint(8, 63);
            runDrive();
            robot.intake.on();
            drive.setTargetPoint(45, 63);
            runDrive();
            drive.forwardByTime(0.2, 1.05);
            runDrive();
            robot.intake.reverse();
            robot.claw.grip();

            // DRIVE TO SCORE CARGO
            drive.setTargetPoint(12, 63);
            runDrive();
            robot.intake.off();
            DriveConstants.admissibleHeadingError = Math.toRadians(10);
            drive.setTargetHeading(Math.toRadians(130));
            drive.setTargetPoint(-3, 37);
            runDrive();

            // DEPOSIT CARGO
//        robot.lift.high();
            robot.arm.high();
            timeout(1);
            robot.claw.deposit();

            // RESET
            robot.claw.timedRetract();
            robot.arm.intake();
//        robot.lift.rest();
            timeout(0.5);
        }

        // PARK
        drive.setTargetHeading(Math.toRadians(180));
        drive.setTargetPoint(16, 63);
        runDrive();
        drive.setTargetPoint(45, 63);
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