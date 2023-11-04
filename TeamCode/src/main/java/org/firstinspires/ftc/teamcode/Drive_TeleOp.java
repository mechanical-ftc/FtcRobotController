package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOP")
public class Drive_TeleOp extends OpMode {


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
        liftRight = hardwareMap.get(DcMotor.class,"Rl");

        servo1 = hardwareMap.get(CRServo.class, "S1");
        servo2 = hardwareMap.get(CRServo.class, "S2");
        servo3 = hardwareMap.get(CRServo.class, "S3");

        intake = hardwareMap.get(DcMotor.class,"I");






//  SET POSITION OF OUR SERVOS EXAMPLE BELOW
//        wideGrabber.setPosition(1);


        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        BlMotor.setDirection(DcMotor.Direction.FORWARD);
        FlMotor.setDirection(DcMotor.Direction.REVERSE);
        BrMotor.setDirection(DcMotor.Direction.REVERSE);
        FrMotor.setDirection(DcMotor.Direction.FORWARD);


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

        double speed = Math.sqrt(2) * Math.pow(Math.pow(gamepad1.left_stick_x, 4) + Math.pow(-gamepad1.left_stick_y, 4), 0.5);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rotation = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        float primaryDiagonalSpeed = (float) (speed * Math.sin(angle - (Math.PI / 4.0)));
        float secondaryDiagonalSpeed = (float) (speed * Math.cos(angle - (Math.PI / 4.0)));

        BlMotor.setPower(secondaryDiagonalSpeed - rotation);
        FrMotor.setPower(secondaryDiagonalSpeed - rotation);
        FlMotor.setPower(primaryDiagonalSpeed + rotation);
        BrMotor.setPower(primaryDiagonalSpeed + rotation);




//        telemetry.addData("Primary:", primaryDiagonalSpeed);
//        telemetry.addData("Secondary:", secondaryDiagonalSpeed);
//        telemetry.update();


        //

        if (gamepad2.left_stick_y > 0.1) {
            liftRight.setPower(gamepad1.left_stick_y);
            liftLeft.setPower(gamepad1.left_stick_y);
        } else if (gamepad2.left_stick_y > -0.1) {
            liftRight.setPower(gamepad1.left_stick_y);
            liftLeft.setPower(gamepad1.left_stick_y);
        } else {
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }

        if (gamepad2.a) {
            servo1.setPower(0.6);
            servo2.setPower(0.6);
            servo3.setPower(0.6);

        }else if (gamepad2.y) {
            servo1.setPower(-0.6);
            servo2.setPower(-0.6);
            servo3.setPower(-0.6);
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





    }


}



