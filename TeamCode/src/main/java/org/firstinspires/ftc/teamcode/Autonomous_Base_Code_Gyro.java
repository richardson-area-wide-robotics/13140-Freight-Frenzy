package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

// Robot Location

@Autonomous(name="Autonomous_Base_Code_Gyro", group="Linear Opmode")
@Disabled
public class Autonomous_Base_Code_Gyro extends LinearOpMode {

    /* Declare OpMode members. */
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

    // Declare Devices
    DcMotor frontleftDrive = null;
    DcMotor frontrightDrive = null;
    DcMotor backleftDrive = null;
    DcMotor backrightDrive = null;
    DcMotor carouselDrive = null;
    Servo servoDrive = null;

    // drive motor position variables
    private int flPos;
    private int frPos;
    private int blPos;
    private int brPos;

    private final double clicksPerInch = 44.563384; // Empirically measured
    private final double tol = .1 * clicksPerInch; // Encoder tolerance


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

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
            gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

            // The right motors need reversing
            frontrightDrive.setDirection(DcMotor.Direction.FORWARD); //used to reverse
            frontleftDrive.setDirection(DcMotor.Direction.REVERSE); // used to forward
            backrightDrive.setDirection(DcMotor.Direction.FORWARD); // used to reverse
            backleftDrive.setDirection(DcMotor.Direction.REVERSE); // used to forward
            carouselDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set the drive motor run modes:
            frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            carouselDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            gyro.calibrate();

            while (!isStopRequested() && gyro.isCalibrating()) {
                sleep(5);
                idle();
            }

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

            while (!isStarted()) {
                telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                telemetry.update();
            }

            gyro.resetZAxisIntegrator();

        }

        // Rotational Speeds
        final double prec = 0.2;
        final double taut = 0.4;
        final double flex = 0.7;
        final double comp = 1.0;

        waitForStart();
        // **Autonomous Steps** ( duckAcc, gyBarArmStrafeR, gyMoveF, gyStrafeR, outTake, returnArm, tiMoveF, tistrafeR )

        // gyMoveF(10, prec);
        duckAcc(30, -1);
        tiMoveF(3, taut);
        tiStrafeR(3, flex, 1);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

    private void duckAcc(int howMuch, int direction) {
        // howMuch is the inches value that will spin the carousel.

        if (opModeIsActive()) {
            // **Adjustable Variables**
            double start = .2; // carousel spinner initial speed
            double step = .5; // frac * this to -> step down speed of accel
            double mach = .5; // maximum speed before duck flies off

            // **Set Variables**
            int duckGoal = (howMuch + Math.abs(carouselDrive.getCurrentPosition()));
            int fracDuckGoal = (carouselDrive.getCurrentPosition() / duckGoal);

            carouselDrive.setTargetPosition(direction * duckGoal);
            while (Math.abs(carouselDrive.getCurrentPosition()) <= duckGoal) {

                carouselDrive.setTargetPosition((int) (Math.min(start + (direction * fracDuckGoal * step), mach)));

            }

        }
    }
/*
    private void gyMoveF(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (howMuch * clicksPerInch);
            int newFLtarget = frontleftDrive.getCurrentPosition() + moveCounts;
            int newFRtarget = frontrightDrive.getCurrentPosition() + moveCounts;
            int newBltarget = backleftDrive.getCurrentPosition() + moveCounts;
            int newBRtarget = backrightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            frontleftDrive.setTargetPosition(newFLtarget);
            frontrightDrive.setTargetPosition(newFRtarget);
            backleftDrive.setTargetPosition(newBltarget);
            backrightDrive.setTargetPosition(newBRtarget);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start robot motion
            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() && (frontleftDrive.isBusy() && frontrightDrive.isBusy() &&
                    backleftDrive.isBusy() && backrightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                Object error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (moveCounts < 0)
                    steer *= -1.0;

                FLSpeed = speed - steer;
                FRSpeed = speed + steer;
                BLSpeed = speed - steer;
                BRSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);

            }

            // fetch motor positions
            flPos = frontleftDrive.getCurrentPosition();
            frPos = frontrightDrive.getCurrentPosition();
            blPos = backleftDrive.getCurrentPosition();
            brPos = backrightDrive.getCurrentPosition();

            // calculate new targets
            flPos += howMuch * clicksPerInch;
            frPos += howMuch * clicksPerInch;
            blPos += howMuch * clicksPerInch;
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


            while (Math.abs(flPos - frontleftDrive.getCurrentPosition()) > tol
                    || Math.abs(frPos - frontrightDrive.getCurrentPosition()) > tol
                    || Math.abs(blPos - backleftDrive.getCurrentPosition()) > tol
                    || Math.abs(brPos - backrightDrive.getCurrentPosition()) > tol) {

                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            // Stop all motion;
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }*/

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
        backleftDrive.setPower(speed);

        frontleftDrive.setTargetPosition(flPos);
        frontrightDrive.setTargetPosition(frPos);
        backleftDrive.setTargetPosition(blPos);
        backrightDrive.setTargetPosition(brPos);


        while (Math.abs(flPos - frontleftDrive.getCurrentPosition()) > tol
                || Math.abs(frPos - frontrightDrive.getCurrentPosition()) > tol
                || Math.abs(blPos - backleftDrive.getCurrentPosition()) > tol
                || Math.abs(brPos - backrightDrive.getCurrentPosition()) > tol) {
            try {
                Thread.sleep(5);
                int flRel = Math.abs(frontleftDrive.getTargetPosition() - frontleftDrive.getCurrentPosition());
                int frRel = Math.abs(frontrightDrive.getTargetPosition() - frontrightDrive.getCurrentPosition());
                int blRel = Math.abs(backleftDrive.getTargetPosition() - backleftDrive.getCurrentPosition());
                int brRel = Math.abs(backrightDrive.getTargetPosition() - backrightDrive.getCurrentPosition());

                int avg = ((flRel + frRel + blRel + brRel) / 4);

                frontleftDrive.setPower(speed * (1 + .01 * (flRel - avg)));
                frontrightDrive.setPower(speed * (1 + .01 * (frRel - avg)));
                backrightDrive.setPower(speed * (1 + .01 * (blRel - avg)));
                backleftDrive.setPower(speed * (1 + .01 * (brRel - avg)));

            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }

    private void outTake(int howLong, double speed) {

        // armIntakeDrive.setPower(speed);

        while (opModeIsActive() && (getRuntime() < howLong)) {

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }

    private void tiMoveF(int howLong, double speed) {

        frontleftDrive.setPower(speed);
        frontrightDrive.setPower(speed);
        backleftDrive.setPower(speed);
        backrightDrive.setPower(speed);

        while (opModeIsActive() && (getRuntime() < howLong)) {

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void tiStrafeR ( int howLong, double speed, int direction ){

        frontleftDrive.setPower(direction * speed);
        frontrightDrive.setPower(-direction * speed);
        backleftDrive.setPower(direction * speed);
        backrightDrive.setPower(-direction * speed);

        while (opModeIsActive() && (getRuntime() < howLong)) {

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
                }
            }
        }
    }