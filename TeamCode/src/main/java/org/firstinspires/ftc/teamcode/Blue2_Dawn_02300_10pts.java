package org.firstinspires.ftc.teamcode;
// NO Proportional
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// Robot Location

@Autonomous(name="Blue2_Dawn_02300_10pts", group="Linear Opmode")

public class Blue2_Dawn_02300_10pts extends LinearOpMode {

    // Declare Devices
    DcMotor frontleftDrive = null;
    DcMotor frontrightDrive = null;
    DcMotor backleftDrive = null;
    DcMotor backrightDrive = null;
    DcMotor carouselDrive = null;
    Servo servoDrive = null;

    // drive motor position variables
    private int flPos; private int frPos; private int blPos; private int brPos;

    private final double clicksPerInch =  44.563384; // Empirically measured
    private final double tol = .1 * clicksPerInch; // Encoder tolerance

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        // Initialize the hardware variables.
        {
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
        }

        // Rotational Speeds
        final double prec = 0.1;
        // **final double taut = 0.4;
        final double flex = 0.7;
        final double comp = 1.0;

        waitForStart();
        // **Autonomous Steps** ( MoveF, StrafeR, RotCl, DelC, ServD )

        strafeR(-32, prec);
        moveF(-50, comp);

    }

    private void moveF(int howMuch, double speed) {
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

            try { Thread.sleep(5); }
            catch (InterruptedException e)
            { e.printStackTrace(); }
        }
    }

    private void strafeR(int howMuch, double speed) {
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
        backleftDrive.setPower(speed*1.11);

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
                } catch (InterruptedException e)
            { e.printStackTrace(); }

        }
    }

    private void rotCl(int whatAngle, double speed) {

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

            try { Thread.sleep(5); }
            catch (InterruptedException e)
            { e.printStackTrace(); }
        }
    }

    private void delClk(int howMuch, double speed) {

        carouselDrive.getCurrentPosition();
        carouselDrive.setPower(speed);
        carouselDrive.setTargetPosition((int) (howMuch * clicksPerInch));

        while (carouselDrive.getCurrentPosition() <= howMuch * clicksPerInch ) {

            try { Thread.sleep(5); }
            catch (InterruptedException e)
            { e.printStackTrace(); }
        }
    }

    private void delCou(int howMuch, double speed) {

        carouselDrive.getCurrentPosition();
        carouselDrive.setPower(speed);
        carouselDrive.setTargetPosition((int) (-howMuch * clicksPerInch));


        while (carouselDrive.getCurrentPosition() >= -howMuch * clicksPerInch ) {

            try { Thread.sleep(5); }
            catch (InterruptedException e)
            { e.printStackTrace(); }
        }
    }

    private void servD() {

        servoDrive.setPosition(-1);

        try { Thread.sleep(1500); }
        catch (InterruptedException e)
        { e.printStackTrace(); }

        servoDrive.setPosition(1);

    }
}