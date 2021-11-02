package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="R2_Nin_22_Auton", group="Linear Opmode")  // @TeleOp(...) is the other common choice

// Start robot with left side against wall & front against end of tile

public class R2_Nin_22_Auton extends LinearOpMode {

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

    // Motor rotation clicks per inch traveled
    private double clicksPerInch =  44.563384; // Empirically measured

    // Rotation speeds
    private double comp = 1; // Complete motor speed
    private double flex = 0.7; // Flexible area to move in motor speed
    private double taut = 0.4; // Taut constraints on movement of motor speed
    private double prec = 0.1; // Precise movements required for motor speed

    // Encoders
    private double tol = .1 * clicksPerInch; // Encoder tolerance

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

        // Steps taken during autonomous
        // Distances in inches, angles in deg, speed
        strafeRPos(1, flex);
        moveForward(1, taut);
        strafeRPos(-1, flex);
        moveForward(1, prec);
        strafeRPos(-1, prec);
        deliveryCar(1, comp);
        strafeRPos(1, flex);
        servoDrop(1, comp);
        strafeRPos(1, flex);
        moveForward(-1, flex);
        strafeRPos(-1, taut);
        moveForward(-1, comp);

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


    private void deliveryCar(int howMuch, double speed) {

        carouselDrive.getCurrentPosition();
        carouselDrive.setTargetPosition((int) (howMuch * clicksPerInch));
        carouselDrive.setPower(speed);

        while (carouselDrive.getCurrentPosition() < howMuch * clicksPerInch ) {

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