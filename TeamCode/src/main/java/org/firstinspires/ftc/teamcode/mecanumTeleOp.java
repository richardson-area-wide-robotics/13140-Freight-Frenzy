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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class mecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontleftDrive = hardwareMap.dcMotor.get("frontleftDrive");
        DcMotor backleftDrive = hardwareMap.dcMotor.get("backleftDrive");
        DcMotor frontrightDrive = hardwareMap.dcMotor.get("frontrightDrive");
        DcMotor backrightDrive = hardwareMap.dcMotor.get("backrightDrive");
        DcMotor carouselDrive = hardwareMap.dcMotor.get("carouselDrive");
        Servo servoDrive = hardwareMap.servo.get("servoDrive");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontleftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backleftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y * 1; // Remember, this is reversed!
            double rx = -gamepad1.right_stick_x;
            double x = 0;
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger < .1 ) {
                x =  gamepad1.left_trigger * 1; // Counteract imperfect strafing
            }
            else if (gamepad1.left_trigger < .1 && gamepad1.right_trigger > 0) {
                x = gamepad1.right_trigger * -1; // Counteract imperfect strafing;
            }
            boolean carouselCounter = gamepad1.cross;
            boolean carouselClock = gamepad1.circle;
            boolean servoDrop = gamepad1.square;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if(carouselClock) {
                carouselDrive.setPower(1);

            }else {

                if (carouselCounter) {
                    carouselDrive.setPower(-1);

                }else {

                    carouselDrive.setPower(0);
                }
            }

            if(servoDrop) {

                servoDrive.setPosition(-1);

            } else {

                servoDrive.setPosition(1);

            }

            frontleftDrive.setPower(frontLeftPower);
            backleftDrive.setPower(backLeftPower);
            frontrightDrive.setPower(frontRightPower);
            backrightDrive.setPower(backRightPower);
        }
    }
}
