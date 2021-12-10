package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// Robot Location

@Autonomous(name = "Autonomous_V4_Revised_Base", group = "Linear Opmode")
public class Autonomous_V5 extends LinearOpMode {

    //setting up cam
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball", "Cube", "Duck", "Marker"};

    private static final String VUFORIA_KEY =
            "AQzpt+D/////AAABmXUuuBKn4UoahBqtNpXN7lJd0jCcYmRl0QmG9GRYDc10Kkjzs6hWFRr31et7XcVTAs17Ndzq2XR3IojRdhnCcvepz1OQYwcH+4Dndj2NpU5Tcl/Rv6/eKbCanlPMmwo7dC73WqPWsMqs/g8q8FMVP7amLMBaoFcTDfgcdJTTzHhaG+coBFFIMIy7rf9SBu30bMy0sRMSCW5XPL99chwgwD8l+wGFQkrLRSP8J27neaL6rn9/F1a2zk17/k+N7+NMS+Wb+qUtvWn0iCNqrsxHASrZqKSloa12K1ZDI0v4pzHFzjOLh1Gd0gGuotmNQ0NhMoU/vDnEXf3Vr7/oCwQ7CYf6FLwKr+RozYf+cIxp70qp";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // Declare Devices
    DcMotor FLDrive = null;
    DcMotor FRDrive = null;
    DcMotor BLDrive = null;
    DcMotor BRDrive = null;

    DcMotor RDuckDrive = null;
    DcMotor BDuckDrive = null;
    DcMotor ArmPivot = null;
    DcMotor InOutTake = null;
    ModernRoboticsI2cGyro gyro = null;

    private final ElapsedTime runtime = new ElapsedTime();

    private final double clicksPerInch = 44.563384; // Empirically measured
    static final double sensitivity = .15; // Larger more responsive but less stable

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
        // Initialize the hardware variables.

        FLDrive = hardwareMap.dcMotor.get("frontleftdrive");
        FRDrive = hardwareMap.dcMotor.get("frontrightdrive");
        BLDrive = hardwareMap.dcMotor.get("backleftdrive");
        BRDrive = hardwareMap.dcMotor.get("backrightdrive");

        RDuckDrive = hardwareMap.dcMotor.get("redcDrive");
        BDuckDrive = hardwareMap.dcMotor.get("bluecDrive");
        ArmPivot = hardwareMap.dcMotor.get("armpivot");
        InOutTake = hardwareMap.dcMotor.get("inouttake");

        ArmPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmPivot.setTargetPosition(0);
        ArmPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmPivot.setPower(1.0);
        ArmPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // The right motors need reversing
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);

        RDuckDrive.setDirection(DcMotor.Direction.FORWARD);
        BDuckDrive.setDirection(DcMotor.Direction.FORWARD);
        ArmPivot.setDirection(DcMotor.Direction.FORWARD);
        InOutTake.setDirection(DcMotor.Direction.FORWARD);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // Set the drive motor run modes:
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RDuckDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BDuckDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        InOutTake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        FLDrive.setTargetPosition(0);
        FRDrive.setTargetPosition(0);
        BLDrive.setTargetPosition(0);
        BRDrive.setTargetPosition(0);

        RDuckDrive.setTargetPosition(0);
        BDuckDrive.setTargetPosition(0);
        ArmPivot.setTargetPosition(0);
        InOutTake.setTargetPosition(0);

        FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RDuckDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BDuckDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        InOutTake.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Rotational Speeds
        // final double prec = 0.2;
        final double taut = 0.4;
        final double flex = 0.7;
        // final double comp = 1.0;

        waitForStart();
        // Position Blocks: ( gyDrive, tiDiagonal )
        // Task Blocks: ( barcode, carousel, fit, outtake )
        gyDrive(10, flex, 0, 1);
        carousel(12, 1);
        tiDiagonal(1, taut, 1, 1, 0);
        fit();
        outtake();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.update();

                    }


                }
            }
        }
    }

    private void initVuforia() {

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "ImagineSight");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
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
        double goalFL = (FLDrive.getCurrentPosition() + (howMuch * clicksPerInch * dir));
        double goalFR = (FRDrive.getCurrentPosition() + (howMuch * clicksPerInch * dir));
        double goalBL = (BLDrive.getCurrentPosition() + (howMuch * clicksPerInch * dir));
        double goalBR = (BRDrive.getCurrentPosition() + (howMuch * clicksPerInch * dir));

        // Set Targets
        FLDrive.setTargetPosition((int) goalFL);
        FRDrive.setTargetPosition((int) goalFR);
        BLDrive.setTargetPosition((int) goalBL);
        BRDrive.setTargetPosition((int) goalBR);

        // Set Mode
        FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Begin Motion
        while (opModeIsActive() && FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy()) {

            FLspeed = power - steer;
            FRspeed = power + steer;
            BLspeed = power - steer;
            BRspeed = power + steer;

            max = Math.min(Math.min(Math.abs(FLspeed), Math.abs(FRspeed)), Math.min(Math.abs(BLspeed), Math.abs(BRspeed)));

            if (max > 1.0) {
                FLspeed /= max;
                FRspeed /= max;
                BLspeed /= max;
                BRspeed /= max;
            }

            FLDrive.setPower(FLspeed);
            FRDrive.setPower(FRspeed);
            BLDrive.setPower(BLspeed);
            BRDrive.setPower(BRspeed);

        }

        // Encoder tolerance
        double tol = .1 * clicksPerInch;
        while (opModeIsActive() && Math.abs(goalFL - FLDrive.getCurrentPosition()) > tol
                || Math.abs(goalFR - FRDrive.getCurrentPosition()) > tol
                || Math.abs(goalBL - BLDrive.getCurrentPosition()) > tol
                || Math.abs(goalBR - BRDrive.getCurrentPosition()) > tol) {

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Stop all motion;
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void tiDiagonal(int howLong, double power, double frontVSback, int left,
                            int right) {
        // Left or Right 0, other 1.
        FLDrive.setPower(power * frontVSback * right);
        FRDrive.setPower(power * frontVSback * left);
        BLDrive.setPower(power * frontVSback * left);
        BRDrive.setPower(power * frontVSback * right);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < howLong)) {

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }

    private void carousel ( int howMuch, double side){
        double ci = .25; // Initial
        double ri = ci; // RED
        double bi = ci; // BLUE
        double s = .05;
        double m = .35; // Maximum Speed

        if(opModeIsActive() && side > 0 && Math.abs(howMuch)>Math.abs(RDuckDrive.getCurrentPosition())) {
            RDuckDrive.setPower(ri);

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if(ri<m){
                ri+=s;
            }

        } else {
            RDuckDrive.setPower(0);
            ri=ci;
        }

        if(opModeIsActive() && 0 > side && Math.abs(howMuch)>Math.abs(BDuckDrive.getCurrentPosition())) {
            BDuckDrive.setPower(bi);

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if(bi<m){
                bi+=s;
            }

        } else {
            BDuckDrive.setPower(0);
            bi= ci;
        }

    }

    private void fit () {

        // Arm Pivot Encoder Levels
        int[] armLevel = {
                0, 40, 720, 790, 900, // 0: Start, 1: In, 2: AL3-Opp, 3: AL2-Opp, 4: AL1-Opp
                120, 190, 290 // 5: AL1-Fnt, 6: AL2-Fnt, 7: AL3-Fnt
        };
        int armPosition = armLevel[0];
        ArmPivot.setTargetPosition(armPosition);
    }

    private void outtake () {
        resetStartTime();
        InOutTake.setPower(1);

        while (opModeIsActive() && runtime.seconds() < 3) {
            try { Thread.sleep(5); }
            catch (InterruptedException e)
            { e.printStackTrace(); }
        }
    }

    // ** PUBLIC VOIDS **
    public double getError ( double targetAngle){

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer ( double error, double sensitivity){

        return Range.clip(error * sensitivity, -1, 1);

    }
}







