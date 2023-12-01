package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc.AHRS;


@TeleOp(name="TeleOP")
public class Drive_TeleOp extends OpMode {
    private AHRS compass = null;

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
    private CRServo servo4 = null;

    private CRServo Shooter = null;

    private DcMotor intake = null;

    private CRServo intakeservo = null;

    private DcMotor Suspension = null;

    @Override
    public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        // Retrieve the motor objects from the hardware map. These names come from the configuration in the robot controller.
        BlMotor = hardwareMap.get(DcMotor.class, "Bl");
        FlMotor = hardwareMap.get(DcMotor.class, "Fl");
        BrMotor = hardwareMap.get(DcMotor.class, "Br");
        FrMotor = hardwareMap.get(DcMotor.class, "Fr");

        liftLeft = hardwareMap.get(DcMotor.class,"Ll");

        servo1 = hardwareMap.get(CRServo.class, "S1");
        servo2 = hardwareMap.get(CRServo.class, "S2");
        servo3 = hardwareMap.get(CRServo.class, "S3");
        servo4 = hardwareMap.get(CRServo.class, "SusExtend");
        Shooter = hardwareMap.get(CRServo.class, "Shooter");

        intake = hardwareMap.get(DcMotor.class,"I");

        intakeservo = hardwareMap.get(CRServo.class, "Iservo");

        Suspension = hardwareMap.get(DcMotor.class,"Sus");







//  SET POSITION OF OUR SERVOS EXAMPLE BELOW
//        wideGrabber.setPosition(1);


        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        BlMotor.setDirection(DcMotor.Direction.REVERSE);
        FlMotor.setDirection(DcMotor.Direction.REVERSE);
        BrMotor.setDirection(DcMotor.Direction.FORWARD);
        FrMotor.setDirection(DcMotor.Direction.FORWARD);


        liftLeft.setDirection(DcMotor.Direction.REVERSE);


        BlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        FlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Suspension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FlMotor.setPower(0);
        FrMotor.setPower(0);
        BlMotor.setPower(0);
        BrMotor.setPower(0);







        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void loop() {

        double left_stick_x = gamepad1.left_stick_x;
        double left_stick_y = gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;
//        double left_stick_x = .5;
//        double left_stick_y = -.5;
//        double right_stick_x = 0;

        double speed = Math.sqrt(2) * Math.pow(Math.pow(left_stick_x, 4) + Math.pow(-left_stick_y, 4), 0.5);
        double angle = Math.atan2(-left_stick_y, -left_stick_x);
        double rotation = Math.signum(right_stick_x) * Math.pow(right_stick_x, 2);

        float primaryDiagonalSpeed = (float) (speed * Math.sin(angle - (Math.PI / 4.0)));
        float secondaryDiagonalSpeed = (float) (speed * Math.cos(angle - (Math.PI / 4.0)));

        telemetry.addData("secondaryDiagonalSpeed: ", secondaryDiagonalSpeed);
        telemetry.addData("primaryDiagonalSpeed", primaryDiagonalSpeed);
        BlMotor.setPower(secondaryDiagonalSpeed + rotation);
        FrMotor.setPower(secondaryDiagonalSpeed - rotation);
        FlMotor.setPower(primaryDiagonalSpeed + rotation);
        BrMotor.setPower(primaryDiagonalSpeed - rotation);




//        telemetry.addData("Primary:", primaryDiagonalSpeed);
//        telemetry.addData("Secondary:", secondaryDiagonalSpeed);
//        telemetry.update();


        //

        if (gamepad2.left_stick_y > 0.1) {
            liftLeft.setPower(0.6);
        } else if (gamepad2.left_stick_y < -0.1) {
            liftLeft.setPower(-0.6);

        } else {
            liftLeft.setPower(0);

        }

        if (gamepad2.a) {
            servo1.setPower(0.3);
            servo2.setPower(0.3);
            servo3.setPower(0.3);

        }else if (gamepad2.y) {
            servo1.setPower(-0.3);
            servo2.setPower(-0.3);
            servo3.setPower(-0.3);
        }else {
            servo1.setPower(0);
            servo2.setPower(0);
            servo3.setPower(0);
        }

        if (gamepad2.left_trigger > 0.1) {
            intake.setPower(gamepad2.left_trigger);
        }else if (gamepad2.right_trigger > 0.1 )  {
            intake.setPower(-gamepad2.right_trigger);
        } else {
            intake.setPower(0);
        }

        if (gamepad2.right_stick_y > 0.1){
            intakeservo.setPower(gamepad2.right_stick_y);
        } else if (gamepad2.right_stick_y < -0.1) {
            intakeservo.setPower(gamepad2.right_stick_y);
        }else {
            intakeservo.setPower(0);
        }

        if (gamepad2.dpad_up) {
            servo4.setPower(1);
        } else if (gamepad2.dpad_down) {
            servo4.setPower(-1);
        }else {
            servo4.setPower(0);
        }

        if (gamepad2.dpad_left) {
            Suspension.setPower(1);
        }else if (gamepad2.dpad_right) {
            Suspension.setPower(-1);
        }else {
            Suspension.setPower(0);
        }

        if (gamepad1.dpad_left) {
            Shooter.setPower(1);
        } else if (gamepad1.dpad_right) {
            Shooter.setPower(-1);
        }else {
            Shooter.setPower(0);
        }


    }



}



