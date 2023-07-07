package org.firstinspires.ftc.teamcode.OpenCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import java.util.ArrayList;
@Autonomous(name = "AprilTagAnonymousInitDetectionExample")
public class testcamera extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    static final double COUNTS_PER_MOTOR_REV = 1440; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double FEET_PER_METER = 3.28084;
    private ElapsedTime runtime = new ElapsedTime();
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
    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 17;
    int MIDDLE = 18;
    int RIGHT = 19;
    // Counter
    int counterLeft = 0;
    int counterMid = 0;
    int counterRight = 0;
    // Parking Spot Decision
    int parkingSpot = 0;
    // Motor stuff 1
    private DcMotor fataDrepta = null;
    private DcMotor fataStanga = null;
    private DcMotor spateDrepta = null;
    private DcMotor spateStanga = null;

    @Override
    public void runOpMode() {
// Motor stuff 2
        fataDrepta = hardwareMap.get(DcMotor.class, "MFD");
        fataStanga = hardwareMap.get(DcMotor.class, "MFS");
        spateDrepta = hardwareMap.get(DcMotor.class, "MSD");
        spateStanga = hardwareMap.get(DcMotor.class, "MSS");
// Motor stuff 3
        fataDrepta.setDirection(DcMotor.Direction.FORWARD);
        fataStanga.setDirection(DcMotor.Direction.FORWARD);
        spateDrepta.setDirection(DcMotor.Direction.FORWARD);
        spateStanga.setDirection(DcMotor.Direction.FORWARD);
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
// If a tag is seen and found, set tag found to true and run following
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
// Since tag found, show to telemetry and run switch
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    switch (tagOfInterest.id) {
                        case 1:
                            LEFT++;
                            break;
                        case 2:
                            MIDDLE++;
                            break;
                        case 3:
                            RIGHT++;
                            break;
                        default:
                    }
                }
// Since tag not found, show to telemetry and other info
                else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
// Tag is not found and run telemetry
            else {
                telemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

            if ((LEFT == 17 || MIDDLE == 18) || RIGHT == 19) {
                if (LEFT == 17) {
                    parkingSpot = 1;
                } else if (MIDDLE == 18) {
                    parkingSpot = 2;
                } else if (RIGHT == 19) {
                    parkingSpot = 3;
                } else {
                    telemetry.addLine("\nSomething is broken!1");
                }
            }

// Update telemetry
            telemetry.update();
            sleep(20);
            if (parkingSpot != 0) {
                break;
            }
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        /* Update the telemetry */
        if (parkingSpot == 1) {
            telemetry.addLine("\nParking in the left spot!");
            driveForward(.5, 10);
            telemetry.addLine("\nDone in the left spot!");
//trajectory
        } else if (parkingSpot == 2) {
            telemetry.addLine("\nParking in the middle spot!");
            driveBack(.5, 50);
            telemetry.addLine("\nDone in the middle spot!");
//trajectory
        } else if (parkingSpot == 3) {
            telemetry.addLine("\nParking in the right spot!");
            driveLeft(.5, 50);
            telemetry.addLine("\nDone in the right spot!");
//trajectory
        } else {
            telemetry.addLine("\nSomething is broken!2 + parkingSpot value is = " + parkingSpot);
//tagToTelemetry(tagOfInterest);
        }
        telemetry.update();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//while (opModeIsActive()) {sleep(20);}
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

    // Next four methods move according to gobilda app assuming encoderDrives work properly, check -inches or -speed
    void driveForward(double speed, double inches) {
        stopResetRunEncoders();
        encoderDriveLF(speed, inches);
        encoderDriveRF(speed, inches);
        encoderDriveLB(speed, inches);
        encoderDriveRB(speed, inches);
    }

    void driveLeft(double speed, double inches) {
        stopResetRunEncoders();
        encoderDriveLF(speed, -inches);
        encoderDriveRF(speed, inches);
        encoderDriveLB(speed, inches);
        encoderDriveRB(speed, -inches);
    }

    void driveRight(double speed, double inches) {
        stopResetRunEncoders();
        encoderDriveLF(speed, inches);
        encoderDriveRF(speed, -inches);
        encoderDriveLB(speed, -inches);
        encoderDriveRB(speed, inches);
    }

    void driveBack(double speed, double inches) {
        stopResetRunEncoders();
        encoderDriveLF(speed, -inches);
        encoderDriveRF(speed, -inches);
        encoderDriveLB(speed, -inches);
        encoderDriveRB(speed, -inches);
    }

    // Stops, resets, and runs using encoders, used before to reset or something idk
    void stopResetRunEncoders() {
        fataDrepta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spateDrepta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fataDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spateDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Taken from github code, takes speed and distance, sets new target value, tells it to run to it, then starts the motion, stops motion, and turns off run to position
    void encoderDriveLF(double speed, double lfInches) {
        int newLeftTarget;
// Ensure that the opmode is still active
        if (opModeIsActive()) {

// Determine new target position, and pass to motor controller
            newLeftTarget = fataStanga.getCurrentPosition() + (int) (lfInches * COUNTS_PER_INCH);
            fataStanga.setTargetPosition(newLeftTarget);
            telemetry.addLine("Current position is " + fataStanga.getCurrentPosition());
            telemetry.addLine("New target position is " + newLeftTarget);
// Turn On RUN_TO_POSITION
            fataStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// reset the timeout time and start motion.
            runtime.reset();
            fataStanga.setPower(Math.abs(speed));
// Stop all motion;
            fataStanga.setPower(0);
// Turn off RUN_TO_POSITION
            fataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// sleep(250); // optional pause after each move
        }
    }

    void encoderDriveRF(double speed, double rfInches) {
        int newRightTarget;
        if (opModeIsActive()) {
            newRightTarget = fataDrepta.getCurrentPosition() + (int) (rfInches * COUNTS_PER_INCH);
            fataDrepta.setTargetPosition(newRightTarget);
            fataDrepta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            fataDrepta.setPower(Math.abs(speed));
            fataDrepta.setPower(0);
            fataDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void encoderDriveLB(double speed, double lbInches) {
        int newLeftTarget;
        if (opModeIsActive()) {
            newLeftTarget = spateStanga.getCurrentPosition() + (int) (lbInches * COUNTS_PER_INCH);
            spateStanga.setTargetPosition(newLeftTarget);
            spateStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            spateStanga.setPower(Math.abs(speed));
            spateStanga.setPower(0);
            spateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void encoderDriveRB(double speed, double rbInches) {
        int newRightTarget;
        if (opModeIsActive()) {
            newRightTarget = spateDrepta.getCurrentPosition() + (int) (rbInches * COUNTS_PER_INCH);
            spateDrepta.setTargetPosition(newRightTarget);
            spateDrepta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            spateDrepta.setPower(Math.abs(speed));
            spateDrepta.setPower(0);
            spateDrepta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}