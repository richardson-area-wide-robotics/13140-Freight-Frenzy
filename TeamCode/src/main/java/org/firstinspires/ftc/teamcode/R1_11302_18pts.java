package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="R1_11302_18pts", group="Linear Opmode")  // @TeleOp(...) is the other common choice

// Start robot with left side against wall & front against end of tile

public class R1_11302_18pts extends LinearOpMode {

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
    private final double clicksPerInch =  44.563384; // Empirically measured

    // Encoders
    private final double tol = .1 * clicksPerInch; // Encoder tolerance

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

        // Rotation speeds
        final double comp = 1; // Complete motor speed
        final double flex = 0.7; // Flexible area to move in motor speed
        final double taut = 0.4; // Taut constraints on movement of motor speed
        final double prec = 0.1; // Precise movements required for motor speed

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Steps taken during autonomous
        // Distances in inches, angles in deg, speed

        strafeRPos(16, flex); // move right: past carousel
        moveForward(12, taut); // move forward: against wall
        strafeRPos(-3, prec); // move left: slight adjustment
        moveForward(3, prec); // move forward: slight adjustment
        deliveryCar(12, comp); // deliver duck: make duck fall
        strafeRPos(16, flex); // move right: into SU
        servoDrop(1, comp); // drop servo: preload drop
        strafeRPos(-8, flex); // move left: park in SU

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

        frontleftDrive.setPower(speed);
        frontrightDrive.setPower(speed);
        backleftDrive.setPower(speed);
        backrightDrive.setPower(speed);

        // move robot to new position
        frontleftDrive.setTargetPosition(flPos);
        frontrightDrive.setTargetPosition(frPos);
        backleftDrive.setTargetPosition(blPos);
        backrightDrive.setTargetPosition(brPos);


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
        frontleftDrive.setPower(speed);
        backrightDrive.setPower(speed);
        frontrightDrive.setPower(speed);
        backleftDrive.setPower(speed);

        frontleftDrive.setTargetPosition(flPos);
        frontrightDrive.setTargetPosition(frPos);
        backleftDrive.setTargetPosition(blPos);
        backrightDrive.setTargetPosition(brPos);


        while ( Math.abs(flPos - frontleftDrive.getCurrentPosition()) > tol
                || Math.abs(frPos - frontrightDrive.getCurrentPosition()) > tol
                || Math.abs(blPos - backleftDrive.getCurrentPosition()) > tol
                || Math.abs(brPos - backrightDrive.getCurrentPosition()) > tol) {
            try {
                Thread.sleep(5);
                int flRel = Math.abs(frontleftDrive.getTargetPosition() - frontleftDrive.getCurrentPosition());
                int frRel = Math.abs(frontrightDrive.getTargetPosition() - frontrightDrive.getCurrentPosition());
                int blRel = Math.abs(backleftDrive.getTargetPosition() - backleftDrive.getCurrentPosition());
                int brRel = Math.abs(backrightDrive.getTargetPosition() - backrightDrive.getCurrentPosition());

                int avg = ((flRel+frRel+blRel+brRel)/4);

                frontleftDrive.setPower(speed*(1+.01*(flRel - avg)));
                frontrightDrive.setPower(speed*(1+.01*(frRel - avg)));
                backrightDrive.setPower(speed*(1+.01*(blRel - avg)));
                backleftDrive.setPower(speed*(1+.01*(brRel - avg)));

            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }

    private void deliveryCar(int howMuch, double speed) {

        carouselDrive.getCurrentPosition();
        carouselDrive.setPower(speed);
        carouselDrive.setTargetPosition((int) (howMuch * clicksPerInch));


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