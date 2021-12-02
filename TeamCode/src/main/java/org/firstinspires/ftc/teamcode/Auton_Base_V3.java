package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Robot Location

@Autonomous(name="Auton_Base_V3", group="Linear Opmode")
@Disabled
public class Auton_Base_V3 extends LinearOpMode {

    // Declare Devices
    DcMotor frontleftDrive = null;
    DcMotor frontrightDrive = null;
    DcMotor backleftDrive = null;
    DcMotor backrightDrive = null;
    DcMotor carouselDrive = null;
    ModernRoboticsI2cGyro gyro = null;
    private ElapsedTime runtime = new ElapsedTime();

    private final double clicksPerInch = 44.563384; // Empirically measured
    private final double tol = .1 * clicksPerInch; // Encoder tolerance
    static final double sensitivity = .15; // Larger more responsive but less stable

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

            // The right motors need reversing
            frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
            frontleftDrive.setDirection(DcMotor.Direction.FORWARD);
            backrightDrive.setDirection(DcMotor.Direction.REVERSE);
            backleftDrive.setDirection(DcMotor.Direction.FORWARD);
            carouselDrive.setDirection(DcMotor.Direction.FORWARD);

            gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

            // Set the drive motor run modes:
            frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            carouselDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.update();

            gyro.calibrate();

            // make sure the gyro is calibrated before continuing
            while (!isStopRequested() && gyro.isCalibrating()) {
                sleep(50);
                idle();
            }

            // Wait for the game to start (Display Gyro value), and reset gyro before we move..
            while (!isStarted()) {
                telemetry.update();
            }

            gyro.resetZAxisIntegrator();

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
        final double prec = 0.2;
        final double taut = 0.4;
        final double flex = 0.7;
        final double comp = 1.0;

        waitForStart();
        // Position Blocks: ( gyDrive, tiDiagonal )
        // Task Blocks: ( barcode, carousel, fit, outtake )

        // Angle of 0 forward?
        // Positive Direction is Clockwise OR CounterClockwise
        // 2.
        // 3.

    }

    private void gyDrive(int howMuch, double power, double angle, double dir) {
        // Variables
        double max;
        double FLspeed;
        double FRspeed;
        double BLspeed;
        double BRspeed;

        double error = getError(angle);
        double steer = getSteer(error, sensitivity);

        // Motor Position Targets
        double goalFL = (frontleftDrive.getCurrentPosition() + (howMuch * clicksPerInch * dir));
        double goalFR = (frontrightDrive.getCurrentPosition() + (howMuch * clicksPerInch * dir));
        double goalBL = (backleftDrive.getCurrentPosition() + (howMuch * clicksPerInch * dir));
        double goalBR = (backrightDrive.getCurrentPosition() + (howMuch * clicksPerInch * dir));

        // Set Targets
        frontleftDrive.setTargetPosition((int) goalFL);
        frontrightDrive.setTargetPosition((int) goalFR);
        backleftDrive.setTargetPosition((int) goalBL);
        backrightDrive.setTargetPosition((int) goalBR);

        // Set Mode
        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Begin Motion
        while (opModeIsActive() && frontleftDrive.isBusy() && frontrightDrive.isBusy() && backleftDrive.isBusy() && backrightDrive.isBusy()) {

            FLspeed = power - steer;
            FRspeed = power + steer;
            BLspeed = power - steer;
            BRspeed = power + steer;

            max = Math.max(Math.max(Math.abs(FLspeed), Math.abs(FRspeed)), Math.max(Math.abs(BLspeed), Math.abs(BRspeed)));

            if (max > 1.0) {
                FLspeed /= max;
                FRspeed /= max;
                BLspeed /= max;
                BRspeed /= max;
            }

            frontleftDrive.setPower(FLspeed);
            frontrightDrive.setPower(FRspeed);
            backleftDrive.setPower(BLspeed);
            backrightDrive.setPower(BRspeed);

        }

        while (opModeIsActive() && Math.abs(goalFL - frontleftDrive.getCurrentPosition()) > tol
                || Math.abs(goalFR - frontrightDrive.getCurrentPosition()) > tol
                || Math.abs(goalBL - backleftDrive.getCurrentPosition()) > tol
                || Math.abs(goalBR - backrightDrive.getCurrentPosition()) > tol) {

            try { Thread.sleep(5); }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Stop all motion;
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void tiDiagonal(int howLong, double power, double frontVSback, int left, int right) {
        // Left or Right 0, other 1.
        frontleftDrive.setPower(power * frontVSback * right);
        frontrightDrive.setPower(power * frontVSback * left);
        backleftDrive.setPower(power * frontVSback * left);
        backrightDrive.setPower(power * frontVSback* right);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < howLong)) {

            try { Thread.sleep(5); }
            catch (InterruptedException e)
            { e.printStackTrace(); }

        }

    }

    private void barcode(){
        // hardwareMap.get(webcam.class, "imagineSight");

        // if(imagineSight)

    }


    private void carousel(int howMuch, double step, double dir) {

        // Variables
        double duckGoal = howMuch*clicksPerInch + carouselDrive.getCurrentPosition();
        double fracDuckGoal = carouselDrive.getCurrentPosition() / duckGoal;
        double start = .1; // Initial Speed
        double mach = .5; // Maximum Speed

        // Logic
        carouselDrive.setTargetPosition((int) (dir * Math.abs(duckGoal)));
        carouselDrive.setPower((int) ( dir * Math.min(start + (fracDuckGoal*step),mach)));

        while (opModeIsActive() && Math.abs(carouselDrive.getCurrentPosition()) <= howMuch * clicksPerInch ) {

            try { Thread.sleep(5); }
            catch (InterruptedException e)
            { e.printStackTrace(); }
        }

    }

    // private void fit{}
    // private void outtake {}


    // ** PUBLIC VOIDS **

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double sensitivity) {
        return Range.clip(error * sensitivity, -1, 1);
    }

}