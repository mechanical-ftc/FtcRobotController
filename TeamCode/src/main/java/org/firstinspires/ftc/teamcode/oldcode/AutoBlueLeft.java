package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.ui.LocalByRefIntentExtraHolder;

@Autonomous(name="AutoBlueLeft")
public class AutoBlueLeft extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //
//    public DcMotor FlMotor;
//    public DcMotor BlMotor;
//    public DcMotor FrMotor;
//    public DcMotor BrMotor;
//    public DcMotor WobbleFlipper;
//    public DcMotor Intake;
//    public DcMotor LeftShooter;
//    public DcMotor RightShooter;
//
//    public Servo WobbleGrabber;
//    public CRServo ConveyorBelt;
//    public Servo WobblePush;

    //28 * 20 / (2ppi * 4.125)
    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //


    public void runOpMode()throws InterruptedException {

        telemetry.clearAll();
        telemetry.addData("Status", "Auto Initialization In Progress");
        telemetry.update();
        //
//        initGyro();
        //
//        BlMotor = hardwareMap.get(DcMotor.class, "Backleft");
//        FlMotor = hardwareMap.get(DcMotor.class, "Frontleft");
//        BrMotor = hardwareMap.get(DcMotor.class, "Backright");
//        FrMotor = hardwareMap.get(DcMotor.class, "Frontright");
//        WobbleFlipper = hardwareMap.get(DcMotor.class, "WobbleFlipper");
//        Intake = hardwareMap.get(DcMotor.class, "Intake");
//        LeftShooter = hardwareMap.get(DcMotor.class, "LeftShooter");
//        RightShooter = hardwareMap.get(DcMotor.class, "RightShooter");
//
//
//        WobbleGrabber = hardwareMap.get(Servo.class, "WobbleGrabber");
//        ConveyorBelt = hardwareMap.get(CRServo.class, "Convey");
//        WobblePush = hardwareMap.get(Servo.class, "Flipper");
//
//        FrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        BrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        RightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        LeftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        RightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        BlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        RingDetector detector = new RingDetector(this);

        //
//        FlMotor.setPower(0);
//        FrMotor.setPower(0);
//        BlMotor.setPower(0);
//        BrMotor.setPower(0);
//        WobbleFlipper.setPower(0);
//        ConveyorBelt.setPower(0);
//        LeftShooter.setPower(0);
//        RightShooter.setPower(0);

        telemetry.clearAll();
        telemetry.addData("Status", "Auto Initialization complete");
        telemetry.update();

//        WobbleGrabber.setPosition(1);

//        initGyro();

        waitForStart();

        int rings = detector.getDecision();
        if (rings == 4) {
//            //moves up to the second box, to perpare for wobble delivery
//            strafeToPosition(-71, 0.8);
//            //loving to the right to line up with the middle of the box
//            moveToPosition(-12.5,0.4);
//            //stops for 1 second
//            sleep(500);
//            //lowers wobble arm
//            WobbleMove(-50, 0.9);
//            //stops robot for 2 seconds
//            sleep(2000);
//            //lets go of wobble
//            WobbleUnGrab();
//            //stops for 1 second
//            sleep(300);
//            //lifts wobble arm back up
//            WobbleMove(40,0.9);
//            //stops for 1 second
//            sleep(900);
////            //
//
//
//            encoderDrive(0.6,-13.32,13.32,2000);
//
//            strafeToPosition(-16,0.6);
//
//
//            moveToPosition(37,0.6);
//
////            moveToPosition(2,0.7);
//
//            WobblePushArmDown();
//            sleep(400);
//
//
//            Shooter();
//            sleep(1500);
//            Shooter();
//            conveyorMove();
//            sleep(3500);
//
//            conveyorMoveStop();
//            StopShooter();
//
//            WobblePushArmUp();
//
////
//            strafeToPosition(-17,0.6);
//
//            moveToPosition(7,0.3);
//
//            moveToPosition(22,0.6);
//
//            moveToPosition(10,0.4);
//
//
//
//            moveToPosition(-1,0.3);
//
//            strafeToPosition(12,0.6);
//
//            moveToPosition(-3,0.4);
//
//            WobblePushArmDown();
//            sleep(400);
//
//            moveToPosition(-55,0.7);
//
//            encoderDrive(0.8,7,-7,2000);
//
//            moveToPosition(-10,0.7);
//
//            moveToPosition(-15,0.9);
//            WobblePushArmUp();
//            sleep(500);
//
//            moveToPosition(29,1);
//

        }

        if (rings == 0) {

//
//            //moves up to the second box, to perpare for wobble delivery
//            strafeToPosition(-9,0.5);
//            strafeToPosition(-30, 0.6);
//            //loving to the right to line up with the middle of the box
//            moveToPosition(-12,0.5);
//            //stops for 1 second
//            sleep(500);
////            lowers wobble arm
//            WobbleMove(-50, 0.8);
//            //stops robot for 2 seconds
//            sleep(2000);
//            //lets go of wobble
//            WobbleUnGrab();
//            //stops for 1 second
//            sleep(500);
//            //lifts wobble arm back up
//            WobbleMove(40,0.8);
//            //stops for 1 second
//            sleep(1000);
////            //
//            strafeToPosition(2,0.3);
//
//
//            encoderDrive(0.4,-13.28,13.28,2000);
//
//            strafeToPosition(-18,0.4);
//
//            sleep(100);
//            moveToPosition(4,0.5);
//
//            sleep(200);
//
//            WobblePushArmDown();
//            sleep(500);
//
//            Shooter();
//            sleep(1500);
//            conveyorMove();
//            Shooter();
//            sleep(3500);
//
//            conveyorMoveStop();
//            StopShooter();
//
//            WobblePushArmUp();
//
//            strafeToPosition(-17,0.5);
//
//            moveToPosition(6,0.3);
//
//            moveToPosition(22,0.6);
//
//            moveToPosition(10,0.4);
//
//            moveToPosition(-1.5,0.5);
//
//            strafeToPosition(14,0.5);
//
//            moveToPosition(-3,0.6);
//
//            sleep(200);
//
//            WobblePushArmDown();
//            sleep(500);
//
//
//
//            encoderDrive(0.6,4.2,-4.2,2000);
//
//            moveToPosition(-41,0.8);
//            WobblePushArmUp();
//            sleep(500);
//
//            moveToPosition(2,0.9);
//
//            strafeToPosition(-20,0.8);
//
//
//
//
//
//        }
//
        }
        if (rings == 1) {


//            //moves up to the second box, to perpare for wobble delivery
//            strafeToPosition(-56, 0.8);
//            //loving to the right to line up with the middle of the box
//            moveToPosition(9,0.3);
//            //stops for 1 second
//            sleep(1000);
//            //lowers wobble arm
//            WobbleMove(-50, 0.8);
//            //stops robot for 2 seconds
//            sleep(2000);
//            //lets go of wobble
//            WobbleUnGrab();
//            //stops for 1 second
//            sleep(1000);
//            //lifts wobble arm back up
//            WobbleMove(40,0.8);
//            //stops for 1 second
//            sleep(1000);
//            //
//
//
//            encoderDrive(0.4,-13,13,2000);
//
//            moveToPosition(24,0.6);
//
//            sleep(1000);
//
//            WobblePushArmDown();
//            sleep(500);
//
//            Shooter();
//            sleep(1000);
//            conveyorMove();
//            Shooter();
//            sleep(5000);
//
//            conveyorMoveStop();
//            StopShooter();
//
//            WobblePushArmUp();
//
//            strafeToPosition(-20,0.5);
//
//            moveToPosition(6,0.4);
//
//            moveToPosition(30,0.6);
//
//            sleep(500);
//
//            strafeToPosition(11.5,0.4);
//
//            WobblePushArmDown();
//
//            moveToPosition(-53,0.5);
//
//            strafeToPosition(10,0.4);
//
//            WobblePushArmUp();
//            sleep(500);
//
//            moveToPosition(4,0.8);
//

////
        }
	//
    }

    //Wobble arm
//    public void WobbleArm(double inches, double speed) {
//        int move =  (int)(Math.round(inches*conversion));
//        WobbleFlipper.setTargetPosition(WobbleFlipper.getCurrentPosition() + move);
//
//        WobbleFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        WobbleFlipper.setPower(speed);
//    }
//
//    // Wobble Grab
//    public void WobbleGrab() {
//        WobbleGrabber.setPosition(1);
//    }
//
//    // Wobble UnGrab
//    public void WobbleUnGrab() {
//        WobbleGrabber.setPosition(0);
//    }
//
//    public void WobbleMove(double inches, double speed) {
//        int move =  (int)(Math.round(inches*conversion));
//
//        WobbleFlipper.setTargetPosition(WobbleFlipper.getCurrentPosition() + move);
//
//        WobbleFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        WobbleFlipper.setPower(speed);
//
//    }
//
//    //
//    /*
//    This function's purpose is simply to drive forward or backward.
//    To drive backward, simply make the inches input negative.
//     */
//    public void moveToPosition(double inches, double speed){
//        //
//        int move = (int)(Math.round(inches*conversion));
//        //
//        BlMotor.setTargetPosition(BlMotor.getCurrentPosition() + move);
//        FlMotor.setTargetPosition(FlMotor.getCurrentPosition() + move);
//        BrMotor.setTargetPosition(BrMotor.getCurrentPosition() + move);
//        FrMotor.setTargetPosition(FrMotor.getCurrentPosition() + move);
//        //
//        FlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //
//        FlMotor.setPower(speed);
//        BlMotor.setPower(speed);
//        FrMotor.setPower(speed);
//        BrMotor.setPower(speed);
//        //
//        while (FlMotor.isBusy() && FrMotor.isBusy() && BlMotor.isBusy() && BrMotor.isBusy()){
//            if (exit){
//                FrMotor.setPower(0);
//                FlMotor.setPower(0);
//                BrMotor.setPower(0);
//                BlMotor.setPower(0);
//                return;
//            }
//        }
//        FrMotor.setPower(0);
//        FlMotor.setPower(0);
//        BrMotor.setPower(0);
//        BlMotor.setPower(0);
//        return;
//    }
//    //
//    /*
//    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
//    Degrees should always be positive, make speedDirection negative to turn left.
//     */
//    public void turnWithGyro(double degrees, double speedDirection){
//        //<editor-fold desc="Initialize">
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double yaw = -angles.firstAngle;//make this negative
//        telemetry.addData("Speed Direction", speedDirection);
//        telemetry.addData("Yaw", yaw);
//        telemetry.update();
//        //
//        telemetry.addData("stuff", speedDirection);
//        telemetry.update();
//        //
//        double first;
//        double second;
//        //</editor-fold>
//        //
//        if (speedDirection > 0){//set target positions
//            //<editor-fold desc="turn right">
//            if (degrees > 10){
//                first = (degrees - 10) + devertify(yaw);
//                second = degrees + devertify(yaw);
//            }else{
//                first = devertify(yaw);
//                second = degrees + devertify(yaw);
//            }
//            //</editor-fold>
//        }else{
//            //<editor-fold desc="turn left">
//            if (degrees > 10){
//                first = devertify(-(degrees - 10) + devertify(yaw));
//                second = devertify(-degrees + devertify(yaw));
//            }else{
//                first = devertify(yaw);
//                second = devertify(-degrees + devertify(yaw));
//            }
//            //
//            //</editor-fold>
//        }
//        //
//        //<editor-fold desc="Go to position">
//        Double firsta = convertify(first - 5);//175
//        Double firstb = convertify(first + 5);//-175
//        //
//        turnWithEncoder(speedDirection);
//        //
//        if (Math.abs(firsta - firstb) < 11) {
//            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        }else{
//            //
//            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        }
//        //
//        Double seconda = convertify(second - 5);//175
//        Double secondb = convertify(second + 5);//-175
//        //
//        turnWithEncoder(speedDirection / 3);
//        //
//        if (Math.abs(seconda - secondb) < 11) {
//            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            FlMotor.setPower(0);
//            FrMotor.setPower(0);
//            BlMotor.setPower(0);
//            BrMotor.setPower(0);
//        }
//        //</editor-fold>
//        //
//        FlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    //
//    /*
//    This function uses the encoders to strafe left or right.
//    Negative input for inches results in left strafing.
//     */
//    public void strafeToPosition(double inches, double speed){
//        //
//        int move = (int)(Math.round(inches * cpi * meccyBias));
//        //
//        BlMotor.setTargetPosition(BlMotor.getCurrentPosition() - move);
//        FlMotor.setTargetPosition(FlMotor.getCurrentPosition() + move);
//        BrMotor.setTargetPosition(BrMotor.getCurrentPosition() + move);
//        FrMotor.setTargetPosition(FrMotor.getCurrentPosition() - move);
//        //
//        FlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //
//        FlMotor.setPower(speed);
//        BlMotor.setPower(speed);
//        FrMotor.setPower(speed);
//        BrMotor.setPower(speed);
//        //
//        while (FlMotor.isBusy() && FrMotor.isBusy() && BlMotor.isBusy() && BrMotor.isBusy()){}
//        FrMotor.setPower(0);
//        FlMotor.setPower(0);
//        BrMotor.setPower(0);
//        BlMotor.setPower(0);
//        return;
//    }
//
//    //
//    /*
//    These functions are used in the turnWithGyro function to ensure inputs
//    are interpreted properly.
//     */
//    public double devertify(double degrees){
//        if (degrees < 0){
//            degrees = degrees + 360;
//        }
//        return degrees;
//    }
//    public double convertify(double degrees){
//        if (degrees > 179){
//            degrees = -(360 - degrees);
//        } else if(degrees < -180){
//            degrees = 360 + degrees;
//        } else if(degrees > 360){
//            degrees = degrees - 360;
//        }
//        return degrees;
//    }
//    //
//    /*
//    This function is called at the beginning of the program to activate
//    the IMU Integrated Gyro.
//     */
//    public void initGyro(){
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        //
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//    }
//    //
//    /*
//    This function is used in the turnWithGyro function to set the
//    encoder mode and turn.
//     */
//    public void turnWithEncoder(double input){
//        FlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        FrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //
//        FlMotor.setPower(input);
//        BlMotor.setPower(input);
//        FrMotor.setPower(-input);
//        BrMotor.setPower(-input);
//    }
//
//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutS) {
//        int newLeftFrontTarget;
//        int newRightFrontTarget;
//        int newLeftBackTarget;
//        int newRightBackTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftFrontTarget = FlMotor.getCurrentPosition() + (int)(leftInches * cpi);
//            newRightFrontTarget = FrMotor.getCurrentPosition() + (int)(rightInches * cpi);
//            newLeftBackTarget  = BlMotor.getCurrentPosition() + (int) (leftInches * cpi);
//            newRightBackTarget = BrMotor.getCurrentPosition() + (int) (rightInches * cpi);
//            FlMotor.setTargetPosition(newLeftFrontTarget);
//            FrMotor.setTargetPosition(newRightFrontTarget);
//            BrMotor.setTargetPosition(newRightBackTarget);
//            BlMotor.setTargetPosition(newLeftBackTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            FlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            FrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            BlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            BrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            FlMotor.setPower(Math.abs(speed));
//            FrMotor.setPower(Math.abs(speed));
//            BrMotor.setPower(Math.abs(speed));
//            BlMotor.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (FrMotor.isBusy() && FlMotor.isBusy() && BlMotor.isBusy() && BrMotor.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newLeftFrontTarget, newRightBackTarget, newRightFrontTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        FlMotor.getCurrentPosition(),
//                        FrMotor.getCurrentPosition(),
//                        BrMotor.getCurrentPosition(),
//                        BlMotor.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            FrMotor.setPower(0);
//            FlMotor.setPower(0);
//            BrMotor.setPower(0);
//            BlMotor.setPower(0);
//            // Turn off RUN_TO_POSITION
//            FlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            FrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            BlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            BrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }
//
//    public void Shooter() {
//        LeftShooter.setPower(-1);
//        RightShooter.setPower(-1);
//    }
//
//    public void conveyorMove() {
//        ConveyorBelt.setPower(-1);
//
//    }
//
//    public void conveyorMoveStop() {
//        ConveyorBelt.setPower(0);
//
//    }
//
//    public void StopShooter() {
//        LeftShooter.setPower(0);
//        RightShooter.setPower(0);
//        ConveyorBelt.setPower(0);
//    }
//
//
//    //
//
//    public void ConveyerbeltSlow() {
//        ConveyorBelt.setPower(-0.4);
//        sleep(2000);
//    }
//
//    public void WobblePushArmDown() {
//        WobblePush.setPosition(180);
//    }
//
//    public void WobblePushArmUp() {
//        WobblePush.setPosition(-180);
//    }
}
