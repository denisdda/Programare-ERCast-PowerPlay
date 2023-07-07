/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.common.value.qual.MinLen;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;


@Autonomous

public class AutoDreapta extends LinearOpMode {
    private DcMotor fataDrepta = null;
    private DcMotor fataStanga = null;
    private DcMotor spateDrepta = null;
    private DcMotor spateStanga = null;
    private Servo Cleste1 = null;
    private Servo Cleste2 = null;
    private DcMotor motorbrat = null;
    private DcMotor motortija = null;


    private ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 2.95;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.40;
    static final double TURN_SPEED = 0.40;
    static final double DIAGONAL_SPEED = 0.40;
    static final double ARM_SPEED = 0.8;
    static final double FORWARD_SPEEDsus = 0.4;
    static final double FORWARD_SPEEDjos = 0.4;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tags of int
    int Unu = 17;
    int Doi = 18;
    int Trei = 19;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        fataDrepta = hardwareMap.get(DcMotor.class, "MFD");
        fataStanga = hardwareMap.get(DcMotor.class, "MFS");
        spateDrepta = hardwareMap.get(DcMotor.class, "MSD");
        spateStanga = hardwareMap.get(DcMotor.class, "MSS");

        Cleste1 = hardwareMap.get(Servo.class, "cleste1");
        Cleste2 = hardwareMap.get(Servo.class, "cleste2");

        motorbrat = hardwareMap.get(DcMotor.class, "MotorBrat");
        motortija = hardwareMap.get(DcMotor.class, "MotorTija");

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Unu) {
                        tagOfInterest = tag;
                        tagFound = true;
                    } else if (tag.id == Doi) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    } else if (tag.id == Trei) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */


        Cleste1.setPosition(0.5);
        Cleste2.setPosition(0);

        sleep(1000);

        motorbrat.setPower(FORWARD_SPEEDsus);
        sleep(1800);

        encoderDrive(DRIVE_SPEED, 4, -4, 4, -4, 4.0);
        encoderDriveDiag(DIAGONAL_SPEED, 35, 35, -35, -35, 6.0);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(800);

    encoderDrive(DRIVE_SPEED, -2, 2, -2, 2, 1);
        Cleste1.setPosition(0);
        Cleste2.setPosition(0.5);
//        encoderDrive(DRIVE_SPEED, -1, 1, -1, 1, 1);
//
//        sleep(800);
//        encoderDriveDiag(DIAGONAL_SPEED, 12.9, 12.9, -12.9, -12.9, 4.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(DRIVE_SPEED, 40, -40, 40, -40, 6.0);
//
//        sleep(1000);
//
//        motorbrat.setPower(FORWARD_SPEEDsus);
//        sleep(1000);
//
//        sleep(300);
//
//        motorbrat.setPower(FORWARD_SPEEDjos);
//        sleep(200);
//
//
//        Cleste1.setPosition(0.5);
//        Cleste2.setPosition(0);
//
//        sleep(1000);
//
//        motorbrat.setPower(FORWARD_SPEEDsus);
//        sleep(1000);
//        motorbrat.setPower(0);
//
//        sleep(1000);
//        encoderDrive(DRIVE_SPEED, -38, 38, -38,38, 6.0);
//        encoderDrive(TURN_SPEED,   35, 35, 35, 35, 4.0);
//
//        motorbrat.setPower(FORWARD_SPEEDsus);
//        sleep(1000);
//
//        Cleste1.setPosition(0);
//        Cleste2.setPosition(0.5);


        sleep(1000);

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (tagOfInterest.id == Doi) {
            encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 4.0);
            encoderDriveDiag(DIAGONAL_SPEED, -12.9, -12.9, 12.9, 12.9, 4.0);  // S1: Forward 47 Inches with 5 Sec timeout
            motorbrat.setPower(-FORWARD_SPEEDsus);
            sleep(1000);
            motorbrat.setPower(0);
        } else if (tagOfInterest.id == Unu) {
            encoderDrive(DRIVE_SPEED, 5, -5, 5, -5, 4.0);
            encoderDriveDiag(DIAGONAL_SPEED, -12.9, -12.9, 12.9, 12.9, 4.0);  // S1: Forward 47 Inches with 5 Sec timeout
            motorbrat.setPower(-FORWARD_SPEEDsus);
            sleep(1000);
            motorbrat.setPower(0);
            encoderDrive(DRIVE_SPEED, 12, -12, 12, -12, 6.0);
        } else if (tagOfInterest.id == Trei) {
            encoderDrive(DRIVE_SPEED, 5, -5, 5, -5, 4.0);
            encoderDriveDiag(DIAGONAL_SPEED, -12.9, -12.9, 12.9, 12.9, 4.0);  // S1: Forward 47 Inches with 5 Sec timeout
            motorbrat.setPower(-FORWARD_SPEEDsus);
            sleep(1000);
            motorbrat.setPower(0);
            encoderDrive(DRIVE_SPEED, -22, 22, -22, 22, 6.0);
        }

    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

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
            fRightTarget = fataDrepta.getCurrentPosition() + (int) (fRightInches * COUNTS_PER_INCH);
            fLeftTarget = fataStanga.getCurrentPosition() + (int) (fleftInches * COUNTS_PER_INCH);
            bRightTarget = spateDrepta.getCurrentPosition() + (int) (bRightInches * COUNTS_PER_INCH);
            bLeftTarget = spateStanga.getCurrentPosition() + (int) (bleftInches * COUNTS_PER_INCH);
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
                telemetry.addData("Running to", " %7d :%7d", fLeftTarget, fRightTarget, bLeftTarget, bRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
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
            newFrontRightTarget = fataDrepta.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = fataStanga.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = spateDrepta.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackLeftTarget = spateStanga.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
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
            motorTarget = motorbrat.getCurrentPosition() + (int) (motorBratInches * 480);
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
            motorbrat.setPower(0);

            // Turn off RUN_TO_POSITION
            motorbrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
