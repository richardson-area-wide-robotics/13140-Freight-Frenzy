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

@Autonomous(name = "AT_R2_W_10", group = "Linear Opmode")
public class AT_R2_W_10 extends LinearOpMode {

    //setting up cam
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball", "Cube", "Duck", "Marker"};

    private static final String VUFORIA_KEY =
            "AYEYxIH/////AAABmVWDEqQv0EXyrybvY1Ci+xEFBepsYnECz7Ua39I5xNbwYAXBQw5iyriVO0+hLn1DGrU81PFuyFVy1/LhN4u/aAp24fKqHIn/oVTbtjWKoDw1IC/IDiCpYDLngQf0YwPRxcx1mfzjwxPFmE2phkDaPL+ebXJWJt1SiXWwNM9rEyd31/xvdfBFWuediDiGpN4+S9zjLUKhnoC5gXZ3zy1jXkiYKRcalP9avwId0Qz2B86nOaiHRWMEnaSn6Gnd6kw4LLwrn9IgdPDLFMPYfTmKOQozr0aX9+Yn+Jj+8JMjKTyvaSo+RYvgtnEzYqqnMKZdVneAt9M0zRErHRT3EbJXzm2/xqH58DZ+vD75+jmNmFBa";

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
        final double prec = 0.2;
        final double taut = 0.4;
        final double flex = 0.7;
        final double comp = 1.0;

        waitForStart();
        // Position Blocks: ( gyDrive, tiDiagonal )
        // Task Blocks: ( barcode, carousel, fit, outtake )
        gyDrive(32, prec, 270, 1);
        gyDrive(-50,flex,180,1);

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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

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

    private void carousel ( int howMuch, double step, double dir){

        // Variables
        double duckGoal = howMuch * clicksPerInch + RDuckDrive.getCurrentPosition();
        double fracDuckGoal = RDuckDrive.getCurrentPosition() / duckGoal;
        double start = .1; // Initial Speed
        double mach = .4; // Maximum Speed

        // Logic
        RDuckDrive.setTargetPosition((int) (dir * Math.abs(duckGoal)));
        RDuckDrive.setPower((int) (dir * Math.min(start + (fracDuckGoal * step), mach)));

        while (opModeIsActive() && Math.abs(RDuckDrive.getCurrentPosition()) <= howMuch * clicksPerInch) {

            try { Thread.sleep(5); }
            catch (InterruptedException e)
            { e.printStackTrace(); }
        }

    }

    private void fit () {

        int[] armLevel = {
                0, 600, 800, 900, // 0: In, 1: AL3-Opp, 2: ShHub-High-Opp, 3: ShHub-Low-Opp
                145, 309, 445, // 4: AL1, 5: AL2, 6: AL3
                850, 750, 550 // 7: AL1-Opp, 8: AL2-Opp, 9: AL3-Opp
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







