package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous
public class PowerPlayAuto2 extends LinearOpMode {

    private ArrayList<AprilTagDetection> latestDetections;

    protected void autoMode(double l) {
        // Red F2, Blue A5 - 90
        // Red F5, Blue A2 - 270
        autoModeLR(l, 90);
    }

    // Red F2
    void autoModeLR(double l, double lr) {
        // 10 inch from target
        turn(0.0);
        detect();
        if (objDetected == 0) {
            objDetected = 1;
        }
        // Move to detect position
        //move( 6, 0.0);
        move(19.25, 0.0);

        turn(lr);

        move(7, lr);

        sleep(500);
        claw.setPosition(0.2);
        sleep(500);

        // Back from cone drop
        move(-8, lr);
        turn(0);

        //driveSpeed1 = DRIVE_SPEED * 1.5;

        //objDetected = 3;
        // Based on detected object, go 1 to 3
        if (objDetected == 1) {
            move(-13, 0.0);

            turn(90);
            move(25.5, 90);

            turn(0);
            move(48, 0);

        } else if (objDetected == 2) {
            // Move back to start position
            move(10, 0);
        } else {
            move(-15, 0.0);
            turn(270);
            move(28, 270);

            turn(0);
            move(45, 0);
        }
    }


    @Override
    public void runOpMode() {

        // ================== end of VISION =========================
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_front = hardwareMap.get(DcMotor.class, "left_front");


        left_front.setPower(left_front.getPower() / 3.6858974359);
        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.FORWARD);
        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (!cameraTest) {
            claw = hardwareMap.get(Servo.class, "claw");
            linearAsDcMotor = hardwareMap.get(DcMotor.class, "arm");

            claw.setPosition(0.5);
        }

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 100);
        RobotLog.ii("IMU", imu.getClass().getName());


        initOpenCV();

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit() && !isStopRequested()) {
            detectCone();
            //detectPos();
            sendTelemetry(true);

            sleep(100);
        }

        if (isStopRequested()) {
            return;
        }


        // Print out how much time we took.
        long end = System.currentTimeMillis();
        sendTelemetry(true);


        // Set the encoders for closed loop speed control, and reset the heading.
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetHeading();

        autoMode(0);

    }

    // ******** Fields and settings

    private DcMotor left_front = null;
    private DcMotor right_front = null;
    private DcMotor left_back = null;
    private DcMotor right_back = null;
    private Servo claw;
    private DcMotor linearAsDcMotor;
    private BNO055IMU imu = null;      // Control/Expansion Hub IMU

    // From move
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    static final double COUNTS_PER_INCH = 40;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.3;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.2;     // Max Turn speed to limit turn rate
    static final double HEADING_THRESHOLD = 0.2;    // How close must the heading get to the target before moving to next step.

    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.01;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.02;     // Larger is more responsive, but also less stable

    static boolean cameraTest = false;

    double driveSpeed1 = DRIVE_SPEED;

    // ********** High level driving ****************

    public void move(double distance, double heading) {
        driveStraight(driveSpeed1, distance, heading);
    }

    public void turn(double heading) {
        turnToHeading(TURN_SPEED, heading);
        holdHeading(TURN_SPEED, heading, 0.6);
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {
        //Orientation orientation = imu.getAngularOrientation();
        //telemetry.addData("Compass", "%.2f Raw: %.2f", robotHeading, orientation.firstAngle);

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
        } else {
            telemetry.addData("Motion", "Turning");
        }
        telemetry.addData("MotorEncoders LF:RF", "%7d:%7d", left_front.getCurrentPosition(), right_front.getCurrentPosition());
        telemetry.addData("MotorEncoders LB:RB", "%7d:%7d", left_back.getCurrentPosition(), right_back.getCurrentPosition());
        telemetry.addData("MotorEncoders LF:RF inch", "%7f:%7f", left_front.getCurrentPosition() / COUNTS_PER_INCH, right_front.getCurrentPosition() / COUNTS_PER_INCH);
        telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);

        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);

        telemetry.addData("Manual F:T:A.", "L: %5.2f : %5.2f R: %5.2f %5.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

        // Provide feedback as to where the robot is located (if we know).
        if (objDetected != 0) {
            telemetry.addData("Object detected", "%d", objDetected);
        }
        if (latestDetections != null ) {
            for (AprilTagDetection det : latestDetections) {
                telemetry.addData("Detected" + det.id, "margin=%4.2f x=%4.2f y=%4.2f pose %4.2f/%4.2f/%4.2f PRY %4.2f/%4.2f %4.2f",
                        det.decisionMargin, det.center.x, det.center.y,
                        det.pose.x, det.pose.y, det.pose.z, det.pose.pitch, det.pose.roll, det.pose.yaw);
            }
        }

        telemetry.update();

    }

    // **********  Low Level driving functions.  ********************

    /**
     * Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTarget = left_front.getCurrentPosition() + moveCounts;
            rightTarget = right_front.getCurrentPosition() + moveCounts;

            RobotLog.ii("DRIVE-S", "%f %f %f TARGET: %d %d REAL: %d %d", maxDriveSpeed, distance, heading,
                    leftTarget, rightTarget, left_front.getCurrentPosition(), right_front.getCurrentPosition());

            // Set Target FIRST, then turn on RUN_TO_POSITION
            left_front.setTargetPosition(leftTarget);
            right_front.setTargetPosition(rightTarget);

            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left_back.setTargetPosition(leftTarget);
            right_back.setTargetPosition(rightTarget);

            left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (left_front.isBusy() && right_front.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            RobotLog.ii("DRIVE-S-DONE", "%f %f HEADING: %f TARGET: %d %d REAL: %d %d", maxDriveSpeed, distance, heading,
                    leftTarget, rightTarget, left_front.getCurrentPosition(), right_front.getCurrentPosition());

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveSide(double maxDriveSpeed,
                          double distance, double heading
    ) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTarget = left_front.getCurrentPosition() + moveCounts;
            rightTarget = right_front.getCurrentPosition() + moveCounts;

            RobotLog.ii("DRIVE-L", "%f %f %f TARGET: %d %d REAL: %d %d", maxDriveSpeed, distance, heading,
                    leftTarget, rightTarget, left_front.getCurrentPosition(), right_front.getCurrentPosition());

            // Set Target FIRST, then turn on RUN_TO_POSITION
            left_front.setTargetPosition(leftTarget);
            right_front.setTargetPosition(-rightTarget);

            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left_back.setTargetPosition(-leftTarget);
            right_back.setTargetPosition(rightTarget);

            left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobotSide(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (left_front.isBusy() && right_front.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobotSide(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            RobotLog.ii("DRIVE-L-DONE", "%f %f %f TARGET: %d %d REAL: %d %d", maxDriveSpeed, distance, heading,
                    leftTarget, rightTarget, left_front.getCurrentPosition(), right_front.getCurrentPosition());
        }

    }
    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        if (Math.abs(drive) > 0.01 && Math.abs(leftTarget - left_front.getCurrentPosition()) < 20) {
            leftSpeed /= 4;
            rightSpeed /= 4;
        }

        left_front.setPower(leftSpeed);
        right_front.setPower(rightSpeed);

        left_back.setPower(leftSpeed);
        right_back.setPower(rightSpeed);

//        RobotLog.ii("MOVE", "(%f %f) HEADING: %f SPEED: %f %f REAL: %d %d IMU: %f %f",
//                drive, turn, robotHeading,
//                leftSpeed, rightSpeed, left_front.getCurrentPosition(), right_front.getCurrentPosition(),
//                imu.getPosition().x, imu.getPosition().y);

    }

    public void moveRobotSide(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        if (Math.abs(drive) > 0.01 && Math.abs(leftTarget - left_front.getCurrentPosition()) < 20) {
            leftSpeed /= 4;
            rightSpeed /= 4;
        }

        left_front.setPower(leftSpeed);
        right_front.setPower(-rightSpeed);

        left_back.setPower(-leftSpeed);
        right_back.setPower(rightSpeed);

        RobotLog.ii("MOVE-L", "(%f %f) HEADING: %f SPEED: %f %f REAL: %d %d", drive, turn, robotHeading,
                leftSpeed, rightSpeed, left_front.getCurrentPosition(), right_front.getCurrentPosition());
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }


    // ================= start of VISION ================================
    // The detected object

    int objDetected = 0;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    private void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName hcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hcam, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.showFpsMeterOnViewport(false);

        RobotLog.ee("OpenCV", "Camera %5.2f %s", camera.getFps(), camera);
        camera.setPipeline(aprilTagDetectionPipeline);

        // Don't show on device screen
        // camera.pauseViewport();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                //camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                RobotLog.ee("OpenCV", "Error %d", errorCode);
            }
        });
    }


    private void detect() {
        long startVision = System.currentTimeMillis();

        objDetected = 0;
        while (System.currentTimeMillis() - startVision < 4500 && objDetected == 0) {
            if (detectCone()) {
                break;
            }
            ;
        }
    }

    private boolean detectCone() {
        latestDetections = aprilTagDetectionPipeline.getLatestDetections();

        // 0 == 1
        // 12 == 3
        // 11 == 2
        for (AprilTagDetection det : latestDetections) {
            if (det.id == 0) {
                objDetected = 1;
                return true;
            }
            if (det.id == 12) {
                objDetected = 3;
                return true;
            }
            if (det.id == 11) {
                objDetected = 2;
                return true;
            }
        }

        return false;
    }


    // ----------------------- END VISION -----------------------------

}
