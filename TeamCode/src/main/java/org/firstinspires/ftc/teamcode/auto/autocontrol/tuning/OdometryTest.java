package org.firstinspires.ftc.teamcode.auto.autocontrol.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

@TeleOp(name = "Odometry Test", group = "TestOpModes")
public class OdometryTest extends LinearOpMode {
    private SwerveDrive swerveDrive;
    private SanfordGyro gyro;
    private ElapsedTime time;
    private double prevTime;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        swerveDrive = new SwerveDrive(hardwareMap);
//        swerveDrive.setPose(-36, -64, -90);
        swerveDrive.setPose(0, 0, 0);
        swerveDrive.setSlowmode(true);
        gyro = new SanfordGyro(hardwareMap);
//        gyro.setStartingAngle(Math.toRadians(-90));
        gyro.setStartingAngle(0);
        time = new ElapsedTime();
        prevTime = 0;

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        waitForStart();
        time.reset();
        while(opModeIsActive()){
            swerveDrive.updatePose(gyro.getAngle(), time.milliseconds() - prevTime);
            prevTime = time.milliseconds();

            double rotation = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            swerveDrive.setMotorPowers(rotation, strafe, forward);

            double x = swerveDrive.getX();
            double y = swerveDrive.getY();
            packet.fieldOverlay()
                    .setStrokeWidth(2)
                    .fillRect(x-6, y-6, 12, 12);
            packet.put("x", x);
            packet.put("y", y);
            packet.put("heading", gyro.getAngle());
            dashboard.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();

            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);
            telemetry.update();
        }
    }
}
