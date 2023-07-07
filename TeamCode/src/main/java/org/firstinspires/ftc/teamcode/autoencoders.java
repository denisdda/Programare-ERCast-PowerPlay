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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoEncoders Test", group="Robot")
//@Disabled
public class autoencoders extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         fataDrepta   = null;
    private DcMotor         fataStanga  = null;
    private DcMotor         spateDrepta   = null;
    private DcMotor         spateStanga  = null;
    private Servo         Cleste1   = null;
    private Servo         Cleste2  = null;
    private DcMotor         motorbrat = null;
    private DcMotor         motortija = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 2.95 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;
    static final double     DIAGONAL_SPEED          = 0.6;
    static final double     ARM_SPEED               = 1;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        fataDrepta  = hardwareMap.get(DcMotor.class, "MFD");
        fataStanga  = hardwareMap.get(DcMotor.class, "MFS");
        spateDrepta  = hardwareMap.get(DcMotor.class, "MSD");
        spateStanga = hardwareMap.get(DcMotor.class, "MSS");

        Cleste1 = hardwareMap.get(Servo.class, "cleste1");
        Cleste2 = hardwareMap.get(Servo.class, "cleste2");

        motorbrat = hardwareMap.get(DcMotor.class, "MotorBrat");
        motortija = hardwareMap.get(DcMotor.class, "MotorTija");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fataDrepta.setDirection(DcMotor.Direction.FORWARD);
        fataStanga.setDirection(DcMotor.Direction.FORWARD);
        spateDrepta.setDirection(DcMotor.Direction.FORWARD);
        spateStanga.setDirection(DcMotor.Direction.FORWARD);

        fataDrepta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spateDrepta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fataDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spateDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                fataDrepta.getCurrentPosition(),
                fataStanga.getCurrentPosition(),
                spateDrepta.getCurrentPosition(),
                fataStanga.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        Cleste1.setPosition(0.5);
        Cleste2.setPosition(0);

        encoderDrive(DRIVE_SPEED,  3.15,-3.15, 3.15, -3.15, 4.0);  // M1: Forward 3.15 Inches
        encoderDriveDiag(DIAGONAL_SPEED,  -25,  -25, 25, 25, 4.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED,  22.5,  -22.5, 22.5, -22.5, 4.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   6.75, 6.75, 6.75, 6.75, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderBrat(ARM_SPEED, -0.2, 4.0);
        encoderDrive(DRIVE_SPEED,  10,  -10, 10, -10, 1.0);
        sleep(1000);

        Cleste1.setPosition(0);
        Cleste2.setPosition(0.5);

        sleep(2000);

//        encoderBrat(ARM_SPEED, 0.2, 1.0);

        motorbrat.setPower(0);

        //        encoderDrive(DRIVE_SPEED,  -10.8,  10.8, -10.8, 10.8, 1.0);
        //        encoderDrive(TURN_SPEED,   -8, -8, -8, -8, 4.0);
        //        encoderDrive(DRIVE_SPEED,  23.5,  -23.5, 23.5, -23.5, 4.0);
        //        sleep(250);
        //
        //        encoderDrive(TURN_SPEED,   18.1, 18.1, 18.1, 18.1, 4.0);
        //        encoderDrive(DRIVE_SPEED,  40,  -40, 40, -40, 4.0);
        //
        //        sleep(1000);
        //        encoderBrat(ARM_SPEED, 0.2, 1.0);
        //        sleep(1000);
        ////        encoderBrat(ARM_SPEED, 0.2, 1.0);
        //        Cleste1.setPosition(0.5);
        //        Cleste2.setPosition(0);
        //
        //        sleep(2000);

//        motorbrat.setPower(0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double fleftInches, double fRightInches, double bleftInches, double bRightInches,
                             double timeoutS) {
        int fLeftTarget;
        int fRightTarget;
        int bLeftTarget;
        int bRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            fRightTarget = fataDrepta.getCurrentPosition() + (int)(fRightInches * COUNTS_PER_INCH);
            fLeftTarget = fataStanga.getCurrentPosition() + (int)(fleftInches * COUNTS_PER_INCH);
            bRightTarget = spateDrepta.getCurrentPosition() + (int)(bRightInches * COUNTS_PER_INCH);
            bLeftTarget = spateStanga.getCurrentPosition() + (int)(bleftInches * COUNTS_PER_INCH);
            fataDrepta.setTargetPosition(fRightTarget);
            fataStanga.setTargetPosition(fLeftTarget);
            spateDrepta.setTargetPosition(bRightTarget);
            spateStanga.setTargetPosition(bLeftTarget);

            // Turn On RUN_TO_POSITION
            fataDrepta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fataStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spateDrepta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spateStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fataDrepta.setPower(Math.abs(speed));
            fataStanga.setPower(Math.abs(speed));
            spateDrepta.setPower(Math.abs(speed));
            spateStanga.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fataDrepta.isBusy() && fataStanga.isBusy() && spateDrepta.isBusy() && spateStanga.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", fLeftTarget,  fRightTarget, bLeftTarget,  bRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        fataDrepta.getCurrentPosition(), fataStanga.getCurrentPosition(), spateDrepta.getCurrentPosition(), spateStanga.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fataDrepta.setPower(0);
            fataStanga.setPower(0);
            spateDrepta.setPower(0);
            spateStanga.setPower(0);

            // Turn off RUN_TO_POSITION
            fataDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spateDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderDriveDiag(double speed,
                                 double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches,
                                 double timeoutSS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontRightTarget = fataDrepta.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = fataStanga.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = spateDrepta.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackLeftTarget = spateStanga.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            fataDrepta.setTargetPosition(newFrontRightTarget);
            fataStanga.setTargetPosition(newFrontLeftTarget);
            spateDrepta.setTargetPosition(newBackRightTarget);
            spateStanga.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            fataDrepta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fataStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spateDrepta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spateStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fataDrepta.setPower(Math.abs(speed));
            fataStanga.setPower(Math.abs(speed));
            spateDrepta.setPower(Math.abs(speed));
            spateStanga.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSS) &&
                    (fataDrepta.isBusy() && fataStanga.isBusy() && spateDrepta.isBusy() && spateStanga.isBusy())) {
                telemetry.update();
            }

            // Stop all motion;
            fataDrepta.setPower(0);
            fataStanga.setPower(0);
            spateDrepta.setPower(0);
            spateStanga.setPower(0);

            // Turn off RUN_TO_POSITION
            fataDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spateDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderBrat(double speed,
                            double motorBratInches,
                            double timeoutSS) {
        int motorTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            motorTarget = motorbrat.getCurrentPosition() + (int)(motorBratInches * COUNTS_PER_INCH);
            motorbrat.setTargetPosition(motorTarget);

            // Turn On RUN_TO_POSITION
            motorbrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorbrat.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSS) &&
                    (motorbrat.isBusy())) {
                telemetry.update();
            }

            // Stop all motion;
//            motorbrat.setPower(0);

            // Turn off RUN_TO_POSITION
            motorbrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
