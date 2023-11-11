package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class firstauto extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor FlMotor;
    public DcMotor BlMotor;
    public DcMotor FrMotor;
    public DcMotor BrMotor;


    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor liftLeft = null;
    private DcMotor liftRight =  null;
    private CRServo servo1 = null;
    private CRServo servo2 = null;
    private CRServo servo3 = null;
    private DcMotor intake = null;
    private CRServo intakeservo = null;


    @Override
    public void runOpMode() throws InterruptedException {
        BlMotor = hardwareMap.get(DcMotor.class, "Bl");
        FlMotor = hardwareMap.get(DcMotor.class, "Fl");
        BrMotor = hardwareMap.get(DcMotor.class, "Br");
        FrMotor = hardwareMap.get(DcMotor.class, "Fr");

        liftLeft = hardwareMap.get(DcMotor.class,"Ll");
        liftRight = hardwareMap.get(DcMotor.class,"Rl");

        servo1 = hardwareMap.get(CRServo.class, "S1");
        servo2 = hardwareMap.get(CRServo.class, "S2");
        servo3 = hardwareMap.get(CRServo.class, "S3");

        intake = hardwareMap.get(DcMotor.class,"I");

        intakeservo = hardwareMap.get(CRServo.class, "Iservo");
    }
}
