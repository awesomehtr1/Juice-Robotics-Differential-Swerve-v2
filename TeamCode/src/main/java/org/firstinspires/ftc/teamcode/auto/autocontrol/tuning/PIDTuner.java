package org.firstinspires.ftc.teamcode.auto.autocontrol.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.autocontrol.MotionProfile;
import org.firstinspires.ftc.teamcode.auto.autocontrol.PDVAController;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;
import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

@Config
@TeleOp(name = "Drive PID Tuner", group = "TestOpModes")
public class PIDTuner extends LinearOpMode {
    private SwerveDrive swerveDrive;
    private ElapsedTime time, timer;
    private SanfordGyro gyro;

    private BasicPID drivePID;
    public static double P;
    public static double D;
    public static double power;

    public static double targetX;
    public static double targetY;

    private double x;
    private double y;

    private double prevTime;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        swerveDrive = new SwerveDrive(hardwareMap);
        swerveDrive.setSlowmode(false);
        gyro = new SanfordGyro(hardwareMap);
        time = new ElapsedTime();
        timer = new ElapsedTime();

        power = 0.3;
        targetX = 0;
        targetY = 0;
        P = 0;
        D = 0;
        prevTime = 0;
        drivePID = new BasicPID(P, 0, D, 0, time);

        waitForStart();
        time.reset();
        timer.reset();
        while (opModeIsActive()) {
            if (timer.seconds() < 3) {
                x = 0;
                y = 0;
            }
            else if (timer.seconds() >= 3 && timer.seconds() < 6) {
                x = targetX;
                y = targetY;
            }
            else if (timer.seconds() >= 6)
                timer.reset();

            drivePID.kP = P;
            drivePID.kD = D;

            swerveDrive.updatePose(gyro.getAngle(), time.milliseconds() - prevTime);
            double distance = distanceBetweenPoints(swerveDrive.getX(), swerveDrive.getY(), x, y);
            double angle = vectorAngle(swerveDrive.getX(), swerveDrive.getY(), x, y);
            drivePID.setState(distance);
            drivePID.updatePID(0);

            double strafe = Math.cos(angle) * drivePID.getPower();
            double forward = Math.sin(angle) * drivePID.getPower();
            if(Math.abs(strafe) > power)
                strafe = Math.signum(strafe) * power;
            if(Math.abs(forward) > power)
                forward = Math.signum(forward) * power;
            swerveDrive.setMotorPowers(0, strafe, forward);

            double xOdo = swerveDrive.getX();
            double yOdo = swerveDrive.getY();
            packet.fieldOverlay()
                    .setStrokeWidth(2)
                    .fillRect(xOdo-6, yOdo-6, 12, 12)
                    .strokeLine(xOdo, yOdo, x, y);
            packet.put("error", distance);
            packet.put("0", 0);
            packet.put("angle", angle);
            dashboard.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
        }
    }

    public double distanceBetweenPoints(double x1, double y1, double x2, double y2) {
        double xDiff = x2 - x1;
        double yDiff = y2 - y1;
        return Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff, 2));
    }

    public double vectorAngle(double x1, double y1, double x2, double y2) {
        double x = x2 - x1;
        double y = y2 - y1;
        return Math.atan2(y, x);
    }
}
