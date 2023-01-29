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
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name = "PowerPlay", group = "Robot")
public class PowerPlay extends LinearOpMode {

    private void autoMode(double l) {

        turn(0.0);

        // To drop the cone
        move( 6, 0.0);
        sleep(2000);

        detect();

        move( 12, 0.0);

        turn(90);

        move(5.5, 90);

        sleep(1000);
        claw.setPosition(0.2);
        sleep(750);

        // Back from cone drop
        move(-5, 90);

        // Move back to start position
        turn(0);

        move(-16, 0);

        // Turn towards parking
        turn(270.0);

        move(26, 270);

        // Move forward
        turn(0.0);

        move(58, 0);

        turn(90.0);
        // Based on detected object, go 1 to 3
        move(36, 90);
    }


    /* Declare OpMode members. */
    private DcMotor left_front = null;
    private DcMotor right_front = null;
    private DcMotor left_back = null;
    private DcMotor right_back = null;
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
    static final double COUNTS_PER_INCH = 40;// (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //(WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.2;     // Max driving speed for better distance accuracy.
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

        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        claw = hardwareMap.get(Servo.class, "claw");
        linearAsDcMotor = hardwareMap.get(DcMotor.class, "arm");


        left_front.setPower(left_front.getPower() / 3.6858974359);

        // Old
        // left_front.setDirection(DcMotor.Direction.FORWARD);
        // right_front.setDirection(DcMotor.Direction.REVERSE);
        // left_back.setDirection(DcMotor.Direction.REVERSE);
        // right_back.setDirection(DcMotor.Direction.FORWARD);
        // New
        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.FORWARD);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 100);
        RobotLog.ii("IMU", imu.getClass().getName());
        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        targets.activate();
        claw.setPosition(0.5);

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

        // left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetHeading();

        // Distance for auto mode.
        double l = 24;

        autoMode = 0;

        int powerset = 0;
        while (opModeIsActive()) {
            // Manual mode
            if (gamepad1.x) {
                autoMode = 0;
                left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                resetHeading();
            }

            // Move to auto-test mode - moves in 1 feet square
            if (gamepad1.b) {
                autoMode = 1;
            }
            if (gamepad2.dpad_down)
            if (autoMode == 0) {
                // Manual controls
                powerset = powerset + 1;
                stickDriveForward = -gamepad1.left_stick_y;
                double horizontal = gamepad1.right_stick_x;
                double vertical = gamepad1.right_stick_y;
                double pivot =  gamepad1.left_stick_x;
                armLevel = gamepad2.right_stick_y;

                turn = gamepad1.right_stick_x;


                if (gamepad1.b) {
                    driveStraight(DRIVE_SPEED, 4, 0);
                }

                // Calibration - move forward 1 foot
                if (gamepad1.y) {
                    turnToHeading(TURN_SPEED, 90);
                }

                int pa = 4;
                //left_front.setPower((pivot + stickDriveForward + turn) / ( pa * left_front_adj));
                //right_front.setPower((pivot + (stickDriveForward - turn)) / pa);

                left_back.setPower((pivot + (stickDriveForward + turn)) / pa);
                right_back.setPower((pivot + stickDriveForward - turn) / (pa));

                left_front.setPower((pivot + stickDriveForward + turn) / (pa));
                right_front.setPower((pivot + stickDriveForward - turn) / pa);

                //left_back.setPower((pivot + vertical + horizontal) / pa);
                //right_back.setPower((pivot + vertical - horizontal)/ pa);

                // if (Pivot != 0) {
                //      turnToHeading(TURN_SPEED, Pivot);
                //      if (autoMode() != 1) {
                //          return ;
                //      }
                //      holdHeading(TURN_SPEED, Pivot, 2);
                //      if (autoMode() != 1) {
                //          return ;
                //      }
                //  }
                /*left_front.setPower(Pivot + stickDriveForward + horizontal);
                right_front.setPower(-Pivot + (stickDriveForward - horizontal));
                left_back.setPower(-Pivot + (stickDriveForward - horizontal));
                right_back.setPower(Pivot + stickDriveForward + horizontal);*/
                if (gamepad2.right_bumper) {
                    claw.setPosition(0.5);
                }
                if (gamepad2.left_bumper) {
                    claw.setPosition(0.3);
                }
                if (gamepad1.right_bumper) {
                    claw.setPosition(0.5);
                }
                if (gamepad1.left_bumper) {
                    claw.setPosition(0.3);
                }
                linearAsDcMotor.setPower(armLevel);
                sendTelemetry(true);
            }

            if (autoMode == 1) {
                autoMode(l);
                autoMode = 1;
            }

        }
    }

    private void squareOld(double l) {
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
            autoMode = 1;
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

    public void move(double distance, double heading) {
        if (autoMode() != 1) {
            return ;
        }
        driveStraight(DRIVE_SPEED, distance, heading);
    }

    public void turn(double heading) {
        if (autoMode() != 1) {
            return ;
        }
        turnToHeading(TURN_SPEED, heading);
        if (autoMode() != 1) {
            return ;
        }
        holdHeading(TURN_SPEED, heading, 1);
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
        if (Math.abs(drive) > 0.01 && Math.abs(leftTarget - left_front.getCurrentPosition()) < 20) {
            leftSpeed /= 4;
            rightSpeed /= 4;
        }

        left_front.setPower(leftSpeed);
        right_front.setPower(rightSpeed);

        left_back.setPower(leftSpeed);
        right_back.setPower(rightSpeed);

        RobotLog.ii("MOVE", "(%f %f) HEADING: %f SPEED: %f %f REAL: %d %d IMU: %f %f %s",
                drive, turn, robotHeading,
                leftSpeed, rightSpeed, left_front.getCurrentPosition(), right_front.getCurrentPosition(),
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
        telemetry.addData("MotorEncoders LF:RF", "%7d:%7d", left_front.getCurrentPosition(), right_front.getCurrentPosition());
        telemetry.addData("MotorEncoders LB:RB", "%7d:%7d", left_back.getCurrentPosition(), right_back.getCurrentPosition());
        telemetry.addData("MotorEncoders LF:RF inch", "%7f:%7f", left_front.getCurrentPosition() / COUNTS_PER_INCH, right_front.getCurrentPosition() / COUNTS_PER_INCH);
        telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);

        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);

        telemetry.addData("Manual F:T:A.", "L: %5.2f : %5.2f R: %5.2f %5.2f",  gamepad1.left_stick_x, gamepad1.left_stick_y,gamepad1.right_stick_x, gamepad1.right_stick_y);

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

    // ================= start of VISION ================================
    // The detected object
    int objDetected = 0;
    String detectDebug = "NOT DETECTED";
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
    private Servo claw;
    private DcMotor linearAsDcMotor;
    private double stickDriveForward;
    private double turn;
    private double armLevel;

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

    // ----------------------- END VISION -----------------------------

}
