package org.firstinspires.ftc.teamcode.auto.autocontrol.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.auto.autocontrol.DriveConstants;
import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.swerve.SwerveConstants;

@TeleOp(name = "Wheel Diameter Tuner", group = "TestOpModes")
public class WheelDiameterTuner extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    AS5600 RFas5600, LFas5600, LBas5600, RBas5600;
    DcMotorEx RF, LF, LB, RB;
    double ticks;
    double inches;

    @Override
    public void runOpMode() throws InterruptedException {
        RFas5600 = new AS5600(hardwareMap, "RFanalog", 2.621, 3.291, 0.001);
        LFas5600 = new AS5600(hardwareMap, "LFanalog", 0.125, 3.286, 0.004);
        LBas5600 = new AS5600(hardwareMap, "LBanalog", 3.251, 3.282, 0.001);
        RBas5600 = new AS5600(hardwareMap, "RBanalog", 2.971, 3.291, 0.0);

        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");

        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        ticks = 0;
        inches = 0;

        waitForStart();
        while(opModeIsActive()) {
            double RFticks = getX(RF.getCurrentPosition(), RFas5600.getAngle());
            double LFticks = getX(LF.getCurrentPosition(), LFas5600.getAngle());
            double LBticks = getX(LB.getCurrentPosition(), LBas5600.getAngle());
            double RBticks = getX(RB.getCurrentPosition(), RBas5600.getAngle());

            ticks = (RFticks+LFticks+LBticks+RBticks) / 4.0;
            inches = ticks / DriveConstants.driveTicksPerInch;

            packet.put("ticks", ticks);
            packet.put("inches", inches);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public double getX(double distance, double angle) {
        return Math.cos(angle) * distance;
    }
}
