package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="blue1WHAuton", group="Linear Opmode")  // @TeleOp(...) is the other common choice
// @Disabled
public class blue1WHAuton extends LinearOpMode {

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
    private double clicksPerInch = 57; // empirically measured
    private double tol = .1 * clicksPerInch; //encoder tolerance

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);


        // Initialize the hardware variables.
        frontleftDrive = hardwareMap.dcMotor.get("frontleftDrive");
        frontrightDrive = hardwareMap.dcMotor.get("frontrightDrive");
        backleftDrive = hardwareMap.dcMotor.get("backleftDrive");
        backrightDrive = hardwareMap.dcMotor.get("backrightDrive");
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

        frontleftDrive.setTargetPosition(0);
        frontrightDrive.setTargetPosition(0);
        backleftDrive.setTargetPosition(0);
        backrightDrive.setTargetPosition(0);
        carouselDrive.setTargetPosition(0);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // *****************Dead reckoning list*************
        // Distances in inches, angles in deg, speed 0.0 to 0.6
        strafeRPos(-16, slow);
        moveForward(20, medium);
        strafeRPos(6, slow);
        deliveryCounter(-30, maximum);
        strafeRPos(-8, slow);
        servoDrop(1, medium);
        moveForward(-120, medium);

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

        while ( Math.abs(flPos - frontleftDrive.getCurrentPosition()) > tol
                || Math.abs(frPos - frontrightDrive.getCurrentPosition()) > tol
                || Math.abs(blPos - backleftDrive.getCurrentPosition()) > tol
                || Math.abs(brPos - backrightDrive.getCurrentPosition()) > tol) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

    }

    private void strafeRPos(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        flPos = frontleftDrive.getCurrentPosition();
        frPos = frontrightDrive.getCurrentPosition();
        blPos = backleftDrive.getCurrentPosition();
        brPos = backrightDrive.getCurrentPosition();

        // calculate new targets
        flPos -= howMuch * clicksPerInch;
        frPos += howMuch * clicksPerInch;
        blPos += howMuch * clicksPerInch;
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

        while ( Math.abs(flPos - frontleftDrive.getCurrentPosition()) > tol
                || Math.abs(frPos - frontrightDrive.getCurrentPosition()) > tol
                || Math.abs(blPos - backleftDrive.getCurrentPosition()) > tol
                || Math.abs(brPos - backrightDrive.getCurrentPosition()) > tol) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }


    private void deliveryCounter(int howMuch, double speed) {

        carouselDrive.getCurrentPosition();
        carouselDrive.setTargetPosition((int) (howMuch * clicksPerInch));
        carouselDrive.setPower(speed);

        while (carouselDrive.getCurrentPosition() > howMuch * clicksPerInch ) {

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }

    private void servoDrop (int howMuch, double speed) {

        servoDrive.setPosition(1);
        servoDrive.setPosition(-1);

        try {
            Thread.sleep(2500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        servoDrive.setPosition(1);

    }
}



    
