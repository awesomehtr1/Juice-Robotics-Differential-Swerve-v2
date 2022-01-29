package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

// high level swerve drive class which utilizes gamepad controls to control the swerve modules
// wraps SwerveKinematics controls and implements SwerveModule
public class SwerveDrive {
    public SwerveKinematics swerveKinematics;

    private HardwareMap hardwareMap;

    private DcMotor RFdrive, LFdrive, LBdrive, RBdrive;
    private DcMotor RFrot, LFrot, LBrot, RBrot;

    // pid constants
    public final double kP = 0.8;
    public final double kI = 0.08;
    public final double kD = 0.011;
    public final double kS = 0.08;
    ElapsedTime RFtime = new ElapsedTime();
    ElapsedTime LFtime = new ElapsedTime();
    ElapsedTime LBtime = new ElapsedTime();
    ElapsedTime RBtime = new ElapsedTime();
    public SwerveRotationPID RFPID, LFPID, LBPID, RBPID;

    private AS5600 RFas5600, LFas5600, LBas5600, RBas5600;

    // array of swerve module objects
    private SwerveModule RF, LF, LB, RB;
    public SwerveModule[] swerveModules = new SwerveModule[4];

    public SwerveDrive(HardwareMap hardwareMap) {
        swerveKinematics = new SwerveKinematics();
        swerveKinematics.setTrackwidth(SwerveConstants.trackwidth);
        swerveKinematics.setWheelbase(SwerveConstants.wheelbase);

        this.hardwareMap = hardwareMap;

        RFdrive = hardwareMap.get(DcMotor.class, "RF");
        LFdrive = hardwareMap.get(DcMotor.class, "LF");
        LBdrive = hardwareMap.get(DcMotor.class, "LB");
        RBdrive = hardwareMap.get(DcMotor.class, "RB");

        //TODO: reverse drive motors if needed
//        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        RFrot = hardwareMap.get(DcMotor.class, "RFrot");
        LFrot = hardwareMap.get(DcMotor.class, "LFrot");
        LBrot = hardwareMap.get(DcMotor.class, "LBrot");
        RBrot = hardwareMap.get(DcMotor.class, "RBrot");

        //TODO: reverse rot motors if needed
        RFrot.setDirection(DcMotorSimple.Direction.REVERSE);
        LFrot.setDirection(DcMotorSimple.Direction.REVERSE);
        LBrot.setDirection(DcMotorSimple.Direction.REVERSE);
        RBrot.setDirection(DcMotorSimple.Direction.REVERSE);

        RFas5600 = new AS5600(hardwareMap, "RFanalog", 2.523);
        LFas5600 = new AS5600(hardwareMap, "LFanalog", 0.133);
        LBas5600 = new AS5600(hardwareMap, "LBanalog", 3.258);
        RBas5600 = new AS5600(hardwareMap, "RBanalog", 2.940);

        RFPID = new SwerveRotationPID(kP, kI, kD, kS, RFtime);
        LFPID = new SwerveRotationPID(kP, kI, kD, kS, LFtime);
        LBPID = new SwerveRotationPID(kP, kI, kD, kS, LBtime);
        RBPID = new SwerveRotationPID(kP, kI, kD, kS, RBtime);

        RF = new SwerveModule(RFrot, RFdrive, RFPID, RFas5600);
        LF = new SwerveModule(LFrot, LFdrive, LFPID, LFas5600);
        LB = new SwerveModule(LBrot, LBdrive, LBPID, LBas5600);
        RB = new SwerveModule(RBrot, RBdrive, RBPID, RBas5600);

        // add swerve modules to array
        swerveModules[0] = RF;
        swerveModules[1] = LF;
        swerveModules[2] = LB;
        swerveModules[3] = RB;
    }

    // sets rotation and drive powers using inputs from gamepad
    public void setMotorPowers(double rotation, double strafe, double forward) {
        swerveKinematics.calculateKinematics(rotation, strafe, forward);
        double[] rotAngleArray = swerveKinematics.getWheelAngles();
        double[] drivePowerArray = swerveKinematics.getWheelVelocities();
        setRotPowerArray(rotAngleArray);
        setDrivePowerArray(drivePowerArray);
    }

    // method for setting pid target for individual swerve module with angle optimization
    public void setRotationPower(SwerveModule module, double angle) {
//        angle = angleOptimization(module, angle);
        module.setAngle(angle);
        double power = module.updatePID(module.getAngle());
        module.setRot(power);
    }

    public void setDrivePower(SwerveModule module, double power) {
        module.setDrive(power);
//        TODO: module.setDrive(power - compensateRotation(module));

    }

    // make pod rotate no more than 90 degrees
    public double angleOptimization(SwerveModule module, double angle) {
        if(Math.abs(angle - module.getAngle()) > Math.PI/2) {
            angle += Math.PI;
            module.setReverseDrive(true);
        }
        else
            module.setReverseDrive(false);
        return angle;
    }

    // compensate wheel rotation when pod rotates
    public double compensateRotation(SwerveModule module) {
        double power = module.getRotPower();
        power *= SwerveConstants.wheelRotPerPodRot;
        power /= SwerveConstants.driveRotRatio;
        return power;
    }

    // sets rotation power for all modules in the array
    // takes 1 dimensional array of rotation angles from SwerveKinematics
    public void setRotPowerArray(double[] rotAngleArray) {
        for (int i = 0; i < 4; i++)
            setRotationPower(swerveModules[i], rotAngleArray[i]);
    }

    // sets drive power for all modules in the array
    // takes 1 dimensional array of drive powers from SwerveKinematics
    public void setDrivePowerArray(double[] drivePowerArray) {
        for (int i = 0; i < 4; i++)
            setDrivePower(swerveModules[i], drivePowerArray[i]);
    }
}
