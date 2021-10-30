package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

    @Autonomous(name="red1BasicAuton", group="Linear Opmode")  // @TeleOp(...) is the other common choice
// @Disabled
    public class red1BasicAuton extends LinearOpMode {

        // Declare Devices
        DcMotor frontleftDrive = null;
        DcMotor frontrightDrive = null;
        DcMotor backleftDrive = null;
        DcMotor backrightDrive = null;
        DcMotor carouselDrive = null;
        Servo servoDrive = null;
        
        // drive motor position variables
        private int flPos; private int frPos; private int blPos; private int brPos;

        // operational constants
        private double maximum = 1; // Save for Carousel Mechanism
        private double fast = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
        private double medium = 0.3; // medium speed
        private double slow = 0.1; // slow speed
        private double time = 6; // time
        private double clicksPerInch = 87.5; // empirically measured
        
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry.setAutoClear(true);


            // Initialize the hardware variables.
            frontleftDrive = hardwareMap.dcMotor.get("leftFront");
            frontrightDrive = hardwareMap.dcMotor.get("rightFront");
            backleftDrive = hardwareMap.dcMotor.get("leftRear");
            backrightDrive = hardwareMap.dcMotor.get("rightRear");
            carouselDrive = hardwareMap.dcMotor.get("carouselDrive");
            servoDrive = hardwareMap.servo.get("servoDrive");

            // The right motors need reversing
            frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
            frontleftDrive.setDirection(DcMotor.Direction.FORWARD);
            backrightDrive.setDirection(DcMotor.Direction.REVERSE);
            backleftDrive.setDirection(DcMotor.Direction.FORWARD);
            carouselDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set the drive motor run modes:
            frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            carouselDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            carouselDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // *****************Dead reckoning list*************
            // Distances in inches, angles in deg, speed 0.0 to 0.6
            strafeRPos(6, fast);
            moveForward(16, medium);
            strafeRPos(-6, medium);
            deliveryCounter(12, maximum);
            strafeRPos(8, fast);
            servoDrop(1,medium);
            strafeRPos(4,slow);
            
        }

        private void moveForward(int howMuch, double speed) {
            // howMuch is in inches. A negative howMuch moves backward.

            // fetch motor positions
            flPos = frontleftDrive.getCurrentPosition();
            frPos = frontrightDrive.getCurrentPosition();
            blPos = backleftDrive.getCurrentPosition();
            brPos = backrightDrive.getCurrentPosition();

            // calculate new targets
            flPos -= howMuch * clicksPerInch;
            frPos -= howMuch * clicksPerInch;
            blPos -= howMuch * clicksPerInch;
            brPos -= howMuch * clicksPerInch;

            // move robot to new position
            frontleftDrive.setTargetPosition(flPos);
            frontrightDrive.setTargetPosition(frPos);
            backleftDrive.setTargetPosition(blPos);
            backrightDrive.setTargetPosition(brPos);
            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(speed);

            // Stop all motion;
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
        }

        private void strafeRPos(int howMuch, double speed) {
            // howMuch is in inches. A negative howMuch moves backward.

            // fetch motor positions
            flPos = frontleftDrive.getCurrentPosition();
            frPos = frontrightDrive.getCurrentPosition();
            blPos = backleftDrive.getCurrentPosition();
            brPos = backrightDrive.getCurrentPosition();

            // calculate new targets
            flPos += howMuch * clicksPerInch;
            frPos -= howMuch * clicksPerInch;
            blPos -= howMuch * clicksPerInch;
            brPos += howMuch * clicksPerInch;

            // move robot to new position
            frontleftDrive.setTargetPosition(flPos);
            frontrightDrive.setTargetPosition(frPos);
            backleftDrive.setTargetPosition(blPos);
            backrightDrive.setTargetPosition(brPos);
            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(speed);

            // Stop all motion;
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);

        }
        
        private void deliveryCounter(int time, double speed) {
            
            carouselDrive.setPower(1);

            // Stop all motion;
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
            carouselDrive.setPower(0);

        }

        private void servoDrop (int howMuch, double speed) throws InterruptedException {
            
            servoDrive.setPosition(1);
            servoDrive.setPosition(-1);
            wait(2);
            servoDrive.setPosition(1);


            // Stop all motion;
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
            carouselDrive.setPower(0);

        }
    }



    
