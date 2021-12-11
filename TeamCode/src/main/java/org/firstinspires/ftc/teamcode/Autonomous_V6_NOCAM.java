package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// Robot Location
@Disabled
@Autonomous(name = "Autonomous_V6_NOCAM", group = "Linear Opmode")
public class Autonomous_V6_NOCAM extends LinearOpMode {

    // Declare Devices
    DcMotor FLDrive = null;
    DcMotor FRDrive = null;
    DcMotor BLDrive = null;
    DcMotor BRDrive = null;

    DcMotor RDuckDrive = null;
    DcMotor BDuckDrive = null;
    DcMotor ArmPivot = null;
    DcMotor InOutTake = null;

    private final ElapsedTime runtime = new ElapsedTime();

    // drive motor position variables
    private int flPos;
    private int frPos;
    private int blPos;
    private int brPos;

    private final double clicksPerInch = 44.563384; // Empirically measured
    private final double tol = .1 * clicksPerInch; // Encoder tolerance

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        // Initialize the hardware variables.

        FLDrive = hardwareMap.dcMotor.get("frontleftdrive");
        FRDrive = hardwareMap.dcMotor.get("frontrightdrive");
        BLDrive = hardwareMap.dcMotor.get("backleftdrive");
        BRDrive = hardwareMap.dcMotor.get("backrightdrive");

        RDuckDrive = hardwareMap.dcMotor.get("redcdrive");
        BDuckDrive = hardwareMap.dcMotor.get("bluecdrive");
        ArmPivot = hardwareMap.dcMotor.get("armpivot");
        InOutTake = hardwareMap.dcMotor.get("inouttake");

        ArmPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmPivot.setTargetPosition(0);
        ArmPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmPivot.setPower(1.0);
        ArmPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // The right motors need reversing
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);

        RDuckDrive.setDirection(DcMotor.Direction.FORWARD);
        BDuckDrive.setDirection(DcMotor.Direction.REVERSE);
        ArmPivot.setDirection(DcMotor.Direction.REVERSE);
        InOutTake.setDirection(DcMotor.Direction.REVERSE);

        // Set the drive motor run modes:
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RDuckDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BDuckDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        InOutTake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        // ARM SET ( 7 fnt, 2 opp )
        // Position Blocks: ( gyDrive, tiDiagonal )
        // Task Blocks: ( carousel, fit, outtake )

    }

    private void moveF(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        flPos = FLDrive.getCurrentPosition();
        frPos = FRDrive.getCurrentPosition();
        blPos = BLDrive.getCurrentPosition();
        brPos = BRDrive.getCurrentPosition();

        // calculate new targets
        flPos += howMuch * clicksPerInch;
        frPos -= howMuch * clicksPerInch;
        blPos -= howMuch * clicksPerInch;
        brPos += howMuch * clicksPerInch;

        // move robot to new position
        FLDrive.setTargetPosition(flPos);
        FRDrive.setTargetPosition(frPos);
        BLDrive.setTargetPosition(blPos);
        BRDrive.setTargetPosition(brPos);
        FLDrive.setPower(speed);
        FRDrive.setPower(speed);
        BLDrive.setPower(speed);
        BRDrive.setPower(speed);


        while (opModeIsActive() && Math.abs(flPos - FLDrive.getCurrentPosition()) > tol
                || Math.abs(frPos - FRDrive.getCurrentPosition()) > tol
                || Math.abs(blPos - BLDrive.getCurrentPosition()) > tol
                || Math.abs(brPos - BRDrive.getCurrentPosition()) > tol) {

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void strafeR(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        flPos = FLDrive.getCurrentPosition();
        frPos = FRDrive.getCurrentPosition();
        blPos = BLDrive.getCurrentPosition();
        brPos = BRDrive.getCurrentPosition();

        // calculate new targets
        flPos += howMuch * clicksPerInch * 1;
        frPos += howMuch * clicksPerInch * 1;
        blPos += howMuch * clicksPerInch * 1.1;
        brPos += howMuch * clicksPerInch * 1.1;

        // move robot to new position
        FLDrive.setPower(speed * 1);
        FRDrive.setPower(speed * 1);
        BRDrive.setPower(speed * 1.1);
        BLDrive.setPower(speed * 1.1);

        FLDrive.setTargetPosition(flPos);
        FRDrive.setTargetPosition(frPos);
        BLDrive.setTargetPosition(blPos);
        BRDrive.setTargetPosition(brPos);


        while (opModeIsActive() && Math.abs(flPos - FLDrive.getCurrentPosition()) > tol
                || Math.abs(frPos - FRDrive.getCurrentPosition()) > tol
                || Math.abs(blPos - BLDrive.getCurrentPosition()) > tol
                || Math.abs(brPos - BRDrive.getCurrentPosition()) > tol) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
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

    private void carousel(int howMuch, double side) {
        double ci = .25; // Initial
        double m = .35; // Maximum Speed

        if (opModeIsActive() && side > 0 && Math.abs(howMuch) > Math.abs(RDuckDrive.getCurrentPosition())) {
            RDuckDrive.setPower(ci);

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (ci < m) {
                ci += .05;
            }

        } else {
            RDuckDrive.setPower(0);
            ci = .25;
        }

        if (opModeIsActive() && 0 > side && Math.abs(howMuch) > Math.abs(BDuckDrive.getCurrentPosition())) {
            BDuckDrive.setPower(ci);

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (ci < m) {
                ci += .05;
            }

        } else {
            BDuckDrive.setPower(0);
            ci = .25;
        }

    }

    private void fit() {

        // Arm Pivot Encoder Levels
        int[] armLevel = {
                0, 40, 720, 790, 900, // 0: Start, 1: In, 2: AL3-Opp, 3: AL2-Opp, 4: AL1-Opp
                120, 190, 290 // 5: AL1-Fnt, 6: AL2-Fnt, 7: AL3-Fnt
        };
        int armPosition = armLevel[0];
        ArmPivot.setTargetPosition(armPosition);
    }

    private void armSet(int pos) {
        int[] armLevel = {
                0, 40, 720, 790, 900, // 0: Start, 1: In, 2: AL3-Opp, 3: AL2-Opp, 4: AL1-Opp
                120, 190, 290 // 5: AL1-Fnt, 6: AL2-Fnt, 7: AL3-Fnt
        };

        int armPosition = armLevel[pos];
        ArmPivot.setTargetPosition(armPosition);

    }

    private void outtake() {
        resetStartTime();
        InOutTake.setPower(1);

        while (opModeIsActive() && runtime.seconds() < 3) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}