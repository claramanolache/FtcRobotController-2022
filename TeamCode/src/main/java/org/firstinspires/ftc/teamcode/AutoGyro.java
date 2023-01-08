/* Copyright (c) 2022 FIRST. All rights reserved.
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;

/**
 * This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 * The code is structured as a LinearOpMode
 * <p>
 * The path to be followed by the robot is built from a series of drive, turn or pause steps.
 * Each step on the path is defined by a single function call, and these can be strung together in any order.
 * <p>
 * The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 * This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 * To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 * <p>
 * This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 * It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 * So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 * See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 * <p>
 * This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 * Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 * <p>
 * Notes:
 * <p>
 * All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 * In this sample, the heading is reset when the Start button is touched on the Driver station.
 * Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clockwise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Control Approach.
 * <p>
 * To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 * <p>
 * Steering power = Heading Error * Proportional Gain.
 * <p>
 * "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 * and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 * <p>
 * "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "0Gyro1", group = "Robot")
public class AutoGyro extends LinearOpMode {
    // The detected object
    int objDetected = 0;
    String detectDebug = "NOT DETECTED";

    // ================= start of VISION ================================
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1",
            "2",
            "3"
    };
    private static final String VUFORIA_KEY =
            "AQfUdZ7/////AAABmTx3uU3M+k/Zgk6XCTX72l94MLmYeIbID02kJBsECtoY0YQHwRWxBjq0p8qlLd5fic84APyKoKoz3+ZYhP5ugl0kMsQBYp+SSFyVdRIfKhaMuJYbMBj6t6P053i50Am0C0WzMJCRiuLa2LILZivOeKIsDCE7ehd8r4wTmBgHq4YTbhbxN+QjiiZHtD9g2UkMn/inNqtOXN8Kcf0k+8KGHyrOjekFWlzPklyfEUbw5EnPWW5nDbzLKg7jTLBAzN7Pv6jDrYNjcrEFqvNqgyn17kwmHq/tN+I2nGctAzGl98EufbcbJW/JJIs/jAOUcrtSAfxssbwnWjgA5Iuz5kshmclDDtqWTqWATu2sYVY+YuN1";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    OpenGLMatrix lastLocation = null;
    private VuforiaTrackables targets;
    private VuforiaTrackable trackable0;
    private ArrayList<VuforiaTrackable> allTrackables;
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;
    private boolean targetVisible;
    private String visibleTarget;
    private int autoMode;

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = true;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters.cameraMonitorViewIdParent = cameraMonitorViewId;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        // Name and locate each trackable object
        identifyTarget(0, "Red Audience Wall", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Red Rear Wall", halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
        identifyTarget(2, "Blue Audience Wall", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Blue Rear Wall", halfField, oneAndHalfTile, mmTargetHeight, 90, 0, -90);

        trackable0 = targets.get(0);
        trackable0.setName("powerPlay"); // can help in debugging; otherwise not necessary


        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

//        for (int i = 0; i < targets.size(); i++) {
//            targets.get(i).setListener(new VuforiaTrackableDefaultListener() {
//                public void onTracked(TrackableResult trackableResult, CameraName cameraName, Camera camera, VuforiaTrackable child) {
//                    super.onTracked(trackableResult, cameraName, camera, child);
//                    RobotLog.ii("TRACKED", "%s %d %f",
//                            trackableResult.getTrackable().getName(), trackableResult.getPose().getData().length, trackableResult.getPose().getData()[0]);
//                }
//            });
//        }
    }


    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void detect() {
        // =====================START VISION==========================

        long posTime = System.currentTimeMillis();
        // if detected remains false after 2 seconds go to the parking
        // place that you dont need to detect for, like the one at the
        // corner or near the wall

        long startVision = System.currentTimeMillis();

        objDetected = 0;
        while (System.currentTimeMillis() - startVision < 2000 && objDetected == 0) {
            if (detectCone()) {
                break;
            }
            ;
        }
        if (objDetected > 0) {
            RobotLog.ii("DETECT", detectDebug);
        }
        // // =========== End VISION ========================
    }

    private boolean detectCone() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                double col = (recognition.getLeft() + recognition.getRight()) / 2;
                double row = (recognition.getTop() + recognition.getBottom()) / 2;
                double width = Math.abs(recognition.getRight() - recognition.getLeft());
                double height = Math.abs(recognition.getTop() - recognition.getBottom());

                objDetected = Integer.parseInt(recognition.getLabel());

                detectDebug = "FOUND " + objDetected + " " +
                        recognition.getLabel() + " " + recognition.getConfidence() * 100;
                // RobotLog.ii("DETECT", "%d %f %s", objDetected, recognition.getConfidence(), recognition.getLabel());

                return true;
            }
        }
        return false;
    }

    private void detectPos() {
        targetVisible = false;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    visibleTarget = trackable.getName();
                }
                break;
            }
        }
    }

    /* Declare OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDriveB = null;
    private DcMotor rightDriveB = null;
    private BNO055IMU imu = null;      // Control/Expansion Hub IMU

    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;


    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 1150; //537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.1;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.1;     // Max Turn speed to limit turn rate
    static final double HEADING_THRESHOLD = 0.2;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.01;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.02;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        // ================== start of VISION ========================
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2, 16.0 / 9.0);
        }

        // ================== end of VISION =========================

        // Initialize the drive system variables.
        leftDriveB = hardwareMap.get(DcMotor.class, "motor2");
        rightDriveB = hardwareMap.get(DcMotor.class, "motor1");
        leftDrive = hardwareMap.get(DcMotor.class, "motor3");
        rightDrive = hardwareMap.get(DcMotor.class, "motor0");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
        rightDriveB.setDirection(DcMotor.Direction.FORWARD);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 100);
        RobotLog.ii("IMU", imu.getClass().getName());
        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        targets.activate();

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

        detect();


        // Print out how much time we took.
        long end = System.currentTimeMillis();
        sendTelemetry(true);


        // Set the encoders for closed loop speed control, and reset the heading.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        // Distance for auto mode.
        double l = 3;

        autoMode = 1;

        while (opModeIsActive()) {
            if (gamepad1.x) {
                autoMode = 0;
            }
            if (gamepad1.a) {
                autoMode++;
            }
            if (autoMode == 0) {
                // Manual controls
                double drive = gamepad1.left_stick_y / 2;
                double turn = gamepad1.right_stick_x / 4;
                driveStraight(0.2, drive, turn);
            }

            if (autoMode == 1) {
                square(l);
            }

        }
    }

    private void square(double l) {
        // Square
        turnToHeading(TURN_SPEED, 0.0);
        if (autoMode() != 1) {
            return;
        }
        holdHeading(TURN_SPEED, 0.0, 2);
        if (autoMode() != 1) {
            return ;
        }
        driveStraight(DRIVE_SPEED, l, 0.0);
        if (autoMode() != 1) {
            return ;
        }
        turnToHeading(TURN_SPEED, 0.0);
        if (autoMode() != 1) {
            return ;
        }
        holdHeading(TURN_SPEED, 0.0, 2);
        if (autoMode() != 1) {
            return ;
        }
        driveStraight(DRIVE_SPEED, -l, 0.0);
        if (autoMode() != 1) {
            return ;
        }

        turnToHeading(TURN_SPEED, 0.0);
        if (autoMode() != 1) {
            return ;
        }
        holdHeading(TURN_SPEED, 0.0, 2);
        if (autoMode() != 1) {
            return ;
        }
        driveStraight(DRIVE_SPEED, l, 0.0);
        if (autoMode() != 1) {
            return ;
        }

        turnToHeading(TURN_SPEED, 90.0);
        if (autoMode() != 1) {
            return ;
        }
        holdHeading(TURN_SPEED, 90.0, 2);
        if (autoMode() != 1) {
            return ;
        }
        driveStraight(DRIVE_SPEED, l, 90.0);
        if (autoMode() != 1) {
            return ;
        }

        turnToHeading(TURN_SPEED, 180.0);
        if (autoMode() != 1) {
            return ;
        }
        holdHeading(TURN_SPEED, 180.0, 2);
        if (autoMode() != 1) {
            return ;
        }
        driveStraight(DRIVE_SPEED, l, 180.0);
        if (autoMode() != 1) {
            return ;
        }
        turnToHeading(TURN_SPEED, 270.0);
        if (autoMode() != 1) {
            return ;
        }
        holdHeading(TURN_SPEED, 270.0, 2);
        if (autoMode() != 1) {
            return ;
        }
        driveStraight(DRIVE_SPEED, l, 270.0);
    }

    int autoMode() {
        if (gamepad1.x) {
            autoMode = 0;
        }
        if (gamepad1.a) {
            autoMode++;
        }
        return autoMode;
    }

    // **********  HIGH Level driving functions.  ********************

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
            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            RobotLog.ii("DRIVE-S", "%f %f %f TARGET: %d %d REAL: %d %d", maxDriveSpeed, distance, heading,
                    leftTarget, rightTarget, leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDriveB.setTargetPosition(leftTarget);
            rightDriveB.setTargetPosition(rightTarget);

            leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

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
                    leftTarget, rightTarget, leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveSide(double maxDriveSpeed,
                          double distance, double heading
    ) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            RobotLog.ii("DRIVE-L", "%f %f %f TARGET: %d %d REAL: %d %d", maxDriveSpeed, distance, heading,
                    leftTarget, rightTarget, leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(-rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDriveB.setTargetPosition(-leftTarget);
            rightDriveB.setTargetPosition(rightTarget);

            leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobotSide(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

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
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            RobotLog.ii("DRIVE-L-DONE", "%f %f %f TARGET: %d %d REAL: %d %d", maxDriveSpeed, distance, heading,
                    leftTarget, rightTarget, leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
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

    // **********  LOW Level driving functions.  ********************

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
        if (Math.abs(drive) > 0.01 && Math.abs(leftTarget - leftDrive.getCurrentPosition()) < 20) {
            leftSpeed /= 4;
            rightSpeed /= 4;
        }

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);

        leftDriveB.setPower(leftSpeed);
        rightDriveB.setPower(rightSpeed);

        RobotLog.ii("MOVE", "(%f %f) HEADING: %f SPEED: %f %f REAL: %d %d IMU: %f %f %s",
                drive, turn, robotHeading,
                leftSpeed, rightSpeed, leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(),
                imu.getPosition().x, imu.getPosition().y, trackable0.getLocation().toString());

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
        if (Math.abs(drive) > 0.01 && Math.abs(leftTarget - leftDrive.getCurrentPosition()) < 20) {
            leftSpeed /= 4;
            rightSpeed /= 4;
        }

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(-rightSpeed);

        leftDriveB.setPower(-leftSpeed);
        rightDriveB.setPower(rightSpeed);

        RobotLog.ii("MOVE-L", "(%f %f) HEADING: %f SPEED: %f %f REAL: %d %d", drive, turn, robotHeading,
                leftSpeed, rightSpeed, leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {
        detectPos();
        Orientation orientation = imu.getAngularOrientation();
        telemetry.addData("Compass", "%.2f Raw: %.2f", robotHeading, orientation.firstAngle);

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
        } else {
            telemetry.addData("Motion", "Turning");
        }
        telemetry.addData("MotorEncoders L:R", "%7d:%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);

        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);


        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("WuPos (inches)", "{X, Y, Z} = %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("WuRot (deg)", "%.0f %s", rotation.thirdAngle, visibleTarget);
        } else {
            telemetry.addData("Visible Target", "none");
        }
        if (objDetected != 0) {
            telemetry.addData("Object detected", "%d %s", objDetected, detectDebug);
        }
        telemetry.update();

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
}
