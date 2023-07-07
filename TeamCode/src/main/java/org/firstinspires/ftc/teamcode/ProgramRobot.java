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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Masina marsava", group="Linear Opmode")
//@Disabled
public class ProgramRobot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fataDreapta = null;
    private DcMotor fataStanga = null;
    private DcMotor spateDreapta = null;
    private DcMotor spateStanga = null;
    private DcMotor motorbrat = null;
    private DcMotor motortija = null;
    private Servo cleste1 = null;
    private Servo cleste2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fataDreapta  = hardwareMap.get(DcMotor.class, "MFD");
        fataStanga = hardwareMap.get(DcMotor.class, "MFS");
        spateDreapta  = hardwareMap.get(DcMotor.class, "MSD");
        spateStanga = hardwareMap.get(DcMotor.class, "MSS");
        motorbrat = hardwareMap.get(DcMotor.class, "MotorBrat");
        motortija = hardwareMap.get(DcMotor.class, "MotorTija");
        cleste1 = hardwareMap.get(Servo.class, "cleste1");
        cleste2 = hardwareMap.get(Servo.class, "cleste2");
        fataDreapta.setDirection(DcMotor.Direction.FORWARD);
        fataStanga.setDirection(DcMotor.Direction.FORWARD);
        spateDreapta.setDirection(DcMotor.Direction.REVERSE);
        spateStanga.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

//            pozition (pozitie, 0.5);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(rx), 1);
            double frontLeftPower = (x + y + rx) / denominator;
            double backLeftPower = (x - y + rx) / denominator;
            double frontRightPower = (x - y - rx) / denominator;
            double backRightPower = (x + y - rx) / denominator;

            fataDreapta.setPower(frontLeftPower);
            fataStanga.setPower(backLeftPower);
            spateDreapta.setPower(frontRightPower);
            spateStanga.setPower(backRightPower);

//            pozitie += (gamepad2.right_trigger-gamepad2.left_trigger);
            double brat = -gamepad2.right_stick_y;

            double putereBrat = (brat);

            motorbrat.setPower(putereBrat);

            if(gamepad2.left_bumper) {

                cleste1.setPosition(0.5);
                cleste2.setPosition(0);
            } else if (gamepad2.right_bumper) {

                cleste1.setPosition(0);
                cleste2.setPosition(0.5);
            }

            if (gamepad2.x) {
                motortija.setPower(1);
            } else if (gamepad2.y) {
                motortija.setPower(-1);
            } else {
                motortija.setPower(0);
            }
        }
    }
}

