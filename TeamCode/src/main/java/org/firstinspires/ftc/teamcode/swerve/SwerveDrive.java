package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

public class SwerveDrive {
    SwerveKinematics swerveKinematics;
    private final double wheelbase = 0.0; //get from cad
    private final double trackwidth = 0.0; //get from cad

    HardwareMap hardwareMap;

    DcMotor LFdrive, RFdrive, RBdrive, LBdrive;
    DcMotorEx LFrot, RFrot, RBrot, LBrot;

    ElapsedTime time = new ElapsedTime();
    // TODO: PID
    public final double kP = 0.0;
    public final double kI = 0.0;
    public final double kD = 0.0;
    public final double kS = 0.0;
    SwerveRotationPID LFPID, RFPID, RBPID, LBPID = new SwerveRotationPID(kP, kI, kD, kS, time);

    SwerveModule LF, RF, RB, LB;
    SwerveModule[] swerveModules = new SwerveModule[4];

    private final double ticksPerRot = 28 * 60 / 30 * 84 / 14; // 28 ticks per rot motor -> 60:30 -> 84:14
    private final double ticksPerRad = ticksPerRot / (Math.PI * 2);

    public SwerveDrive(HardwareMap hardwareMap) {
        swerveKinematics = new SwerveKinematics();
        swerveKinematics.setTrackwidth(trackwidth);
        swerveKinematics.setWheelbase(wheelbase);

        this.hardwareMap = hardwareMap;

        LFdrive = hardwareMap.get(DcMotor.class, "LF");
        RFdrive = hardwareMap.get(DcMotor.class, "RF");
        RBdrive = hardwareMap.get(DcMotor.class, "RB");
        LBdrive = hardwareMap.get(DcMotor.class, "LB");

        LFrot = hardwareMap.get(DcMotorEx.class, "LFrot");
        RFrot = hardwareMap.get(DcMotorEx.class, "RFrot");
        RBrot = hardwareMap.get(DcMotorEx.class, "RBrot");
        LBrot = hardwareMap.get(DcMotorEx.class, "LBrot");

        LF = new SwerveModule(LFrot, LFdrive, LFPID);
        RF = new SwerveModule(RFrot, RFdrive, RFPID);
        RB = new SwerveModule(RBrot, RBdrive, RBPID);
        LB = new SwerveModule(LBrot, LBdrive, LBPID);

        swerveModules[0] = LF;
        swerveModules[1] = RF;
        swerveModules[2] = RB;
        swerveModules[3] = LB;
    }

    public void setMotorPowers(double rotation, double strafe, double forward) {
        swerveKinematics.calculateKinematics(rotation, strafe, forward);
        double[] rotAngleArray = swerveKinematics.getWheelAngles();
        double[] drivePowerArray = swerveKinematics.getWheelVelocities();
        setRotPowerArray(rotAngleArray);
        setDrivePowerArray(drivePowerArray);
    }

    public void setRotationPower(SwerveModule module, double angle) {
        module.setAngle(angle);
        double power = module.updatePID(module.getAngle());
        module.setRot(power);
    }

    public void setDrivePower(SwerveModule module, double power) {
        module.setDrive(power);
    }

    public void setRotPowerArray(double[] rotAngleArray) {
        for (int i = 0; i < 4; i++)
            setRotationPower(swerveModules[i], rotAngleArray[i]);
    }

    public void setDrivePowerArray(double[] drivePowerArray) {
        for (int i = 0; i < 4; i++)
            setDrivePower(swerveModules[i], drivePowerArray[i]);
    }
}
