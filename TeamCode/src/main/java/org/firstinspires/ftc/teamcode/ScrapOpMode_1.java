/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ScrapOpMode_1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Declare Motors.
        DcMotor FLDrive = hardwareMap.dcMotor.get("frontleftDrive");
        DcMotor FRDrive = hardwareMap.dcMotor.get("frontrightDrive");
        DcMotor BLDrive = hardwareMap.dcMotor.get("backleftDrive");
        DcMotor BRDrive = hardwareMap.dcMotor.get("backrightDrive");

        DcMotor RDuckDrive = hardwareMap.dcMotor.get("redcdrive");
        DcMotor BDuckDrive = hardwareMap.dcMotor.get("bluecdrive");
        DcMotor ArmPivot = hardwareMap.dcMotor.get("armpivot");
        DcMotor InOutTake = hardwareMap.dcMotor.get("inouttake");

        // Arm Stay up Logic
        ArmPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmPivot.setTargetPosition(0);
        //ArmPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //ArmPivot.setPower(1.0);
        //ArmPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Reverse Necessary Motors. (Right)
        FLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        FRDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        BLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BRDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        RDuckDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        BDuckDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        InOutTake.setDirection(DcMotorSimple.Direction.REVERSE);

        int armPosition = 0;

        // Wait for Match Start.
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // // //  Section #1: Movement // // //

            // Variable Assignments.
            double x = gamepad1.left_stick_x/-1; // Strafing
            double y = gamepad1.left_stick_y; // Forwards & Back
            double rx = -gamepad1.right_stick_x; // Rotation

            // Input to Power Conversion.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLPower = (y + x + rx) / denominator;
            double FRPower = (y - x - rx) / denominator;
            double BLPower = (y - x + rx) / denominator;
            double BRPower = (y + x - rx) / denominator;

            // Power to Output Command.
            FLDrive.setPower(FLPower);
            FRDrive.setPower(FRPower);
            BLDrive.setPower(BLPower);
            BRDrive.setPower(BRPower);

            //Arm encoder counting
            /*if (gamepad1.left_bumper)
            {
                armPosition += 10;
            }
            else if (gamepad1.right_bumper)
            {
                armPosition -= 10;
            }
            else
                armPosition = ArmPivot.getCurrentPosition();
            ArmPivot.setTargetPosition(armPosition);
*/
            if (gamepad1.left_bumper)
            {
                ArmPivot.setPower(1.0);
            }
            else if (gamepad1.right_bumper)
            {
                ArmPivot.setPower(-1.0);
            }
            else
            {
                ArmPivot.setPower(0.0);
            }


            // Intake / Outtake Logic.
            if(gamepad1.left_trigger > .1) {
                InOutTake.setPower(1);
            } else if (gamepad1.right_trigger > .1) {
                InOutTake.setPower(-1);
            } else { InOutTake.setPower(0); }
            telemetry.addData("Encoder value", ArmPivot.getCurrentPosition());
            telemetry.update();

        }
    }
}