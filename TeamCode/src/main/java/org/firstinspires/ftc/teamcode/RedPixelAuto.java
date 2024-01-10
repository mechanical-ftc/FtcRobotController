package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="RedPixelAuto", group="chad")
public class RedPixelAuto extends LinearOpMode {


    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor backleft = null;
    private DcMotor backright = null;

    private DcMotor liftLeft = null;

    private CRServo servo1 = null;
    private CRServo servo2 = null;
    private CRServo servo3 = null;

    private DcMotor intake = null;

    private CRServo intakeservo = null;

    private DcMotor Suspension = null;
    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    // strafing movement
    //
    Double meccyBias = 0.6;

    Double conversion = cpi * bias;
    Boolean exit = false;

    public void runOpMode(){
        //
        //
        backleft = hardwareMap.get(DcMotor.class, "Bl");
        frontleft = hardwareMap.get(DcMotor.class, "Fl");
        backright = hardwareMap.get(DcMotor.class, "Br");
        frontright = hardwareMap.get(DcMotor.class, "Fr");

        liftLeft = hardwareMap.get(DcMotor.class,"Ll");

        servo1 = hardwareMap.get(CRServo.class, "S1");
        servo2 = hardwareMap.get(CRServo.class, "S2");
        servo3 = hardwareMap.get(CRServo.class, "S3");

        intake = hardwareMap.get(DcMotor.class,"I");

        intakeservo = hardwareMap.get(CRServo.class, "Iservo");

        Suspension = hardwareMap.get(DcMotor.class,"Sus");


        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.FORWARD);


        liftLeft.setDirection(DcMotor.Direction.FORWARD);


        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Suspension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);



        waitForStart();
        //
        moveToPosition(-43, 0.2);
        strafeToPosition(55,0.2);
        Lifty(17,0.4);
        sleep(3000);
        disy(-0.4);
        sleep(3000);
        disy(0);
        Lifty(-10, 0.3);
        sleep(3000);
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition()  + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() - move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }
    //
    /*

    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }

    public void Lifty(double inches, double speed) {
        int move = (int)(Math.round(inches * cpi * meccyBias));
        liftLeft.setTargetPosition(liftLeft.getCurrentPosition() + move);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(speed);
    }

    public void disy(double speed) {
        servo1.setPower(speed);
        servo2.setPower(speed);
        servo3.setPower(speed);
    }
    //

    public static class RedFarPark {
    }
}
