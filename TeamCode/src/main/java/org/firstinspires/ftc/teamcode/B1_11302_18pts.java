package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="B1_11302_18pts", group="Linear Opmode")  // @TeleOp(...) is the other common choice

// Start robot with front against wall & left side against tile

public class B1_11302_18pts extends LinearOpMode {

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
        final double prec = 0.2; // Precise movements required for motor speed

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Steps taken during autonomous
        // Distances in inches, angles in deg, speed
        moveForward(-18, flex); // move backward: past carousel
        strafeRPos(-30, prec); // move left: against wall
        moveForward(3,flex);
        strafeRPos(-3,prec);
        deliveryCar(34, comp); // deliver duck: make duck fall
        strafeRPos(2, prec);
        moveForward(-16, flex); // move backward: into SU
        servoDrop(1, comp); // drop servo: preload drop

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

    private void turnClockwise(int whatAngle, double speed) {
        // whatAngle is in degrees. A negative whatAngle turns counterclockwise.
        final double clicksPerDeg =  1.555556; // 560 clicks / 360 deg

        // fetch motor positions
        flPos = frontleftDrive.getCurrentPosition();
        frPos = frontrightDrive.getCurrentPosition();
        blPos = backleftDrive.getCurrentPosition();
        brPos = backrightDrive.getCurrentPosition();

        // calculate new targets
        flPos += whatAngle * clicksPerDeg;
        frPos -= whatAngle * clicksPerDeg;
        brPos += whatAngle * clicksPerDeg;
        brPos -= whatAngle * clicksPerDeg;

        // move robot to new position
        frontleftDrive.setTargetPosition(flPos);
        frontrightDrive.setTargetPosition(frPos);
        backleftDrive.setTargetPosition(brPos);
        backrightDrive.setTargetPosition(brPos);
        frontleftDrive.setPower(speed);
        frontrightDrive.setPower(speed);
        backleftDrive.setPower(speed);
        backrightDrive.setPower(speed);

        while (frontleftDrive.getCurrentPosition() < whatAngle * clicksPerDeg
                || frontrightDrive.getCurrentPosition() < whatAngle * clicksPerDeg
                || backleftDrive.getCurrentPosition() < whatAngle * clicksPerDeg
                || backrightDrive.getCurrentPosition() < whatAngle * clicksPerDeg) {

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void deliveryCar(int howMuch, double speed) {

        carouselDrive.getCurrentPosition();
        carouselDrive.setPower(speed);
        carouselDrive.setTargetPosition((int) (howMuch * clicksPerInch));


        while (carouselDrive.getCurrentPosition() >= howMuch * clicksPerInch ) {

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
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        servoDrive.setPosition(1);

    }
}