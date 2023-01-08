package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;

@TeleOp(name = "PowerPlayAutonomous")
public class PowerPlayAutonomous extends LinearOpMode {

    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;

    DcMotor carouselMotor;
    DcMotor armMotor;
    DcMotor intakeMotor;

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

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = true;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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

        while (System.currentTimeMillis() - startVision < 2000 && objDetected == 0) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                    objDetected = Integer.parseInt(recognition.getLabel());

                    detectDebug = "FOUND " + objDetected + " " +
                            recognition.getLabel() + " " + recognition.getConfidence() * 100;
                    break;
                }
                telemetry.update();
            }
        }
        // // =========== End VISION ========================

    }

    // ================== end of VISION =============================

    // Resets the encoder position of the given "motor" and sets it to go
    // to the given "position".
    void setEncoderPosition(DcMotor motor, int position) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Resets the encoder for all the 4 motors and sets their target positions.
    void setEncoderPositions(int leftFrontPosition, int leftBackPosition,
                             int rightFrontPosition, int rightBackPosition) {
        setEncoderPosition(leftFrontMotor, leftFrontPosition);
        setEncoderPosition(leftBackMotor, leftBackPosition);
        setEncoderPosition(rightFrontMotor, rightFrontPosition);
        setEncoderPosition(rightBackMotor, rightBackPosition);
    }

    void stopUsingEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Sets the powers for all 4 motors.
    void setPowers(double leftFrontPower, double leftBackPower,
                   double rightFrontPower, double rightBackPower) {
        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    // void backupIntoWall(int milliseconds) {
    //   stopUsingEncoders();
    //   setPowers(0.25, 0.25, -0.25, -0.25);
    //   sleep(milliseconds);
    //   stopAllMotors();
    // }

    // Stops all the drive train motors.
    void stopAllMotors() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Wait for at least 1 motor to become idle.
    void waitForIdleMotor() {
        while (opModeIsActive() && leftFrontMotor.isBusy()
                && leftBackMotor.isBusy() && rightFrontMotor.isBusy()
                && rightBackMotor.isBusy()) {
            // Put run blocks here.
            String data = String.format("Left front: %d busy = %b, left back: %d " +
                            "busy = %b, right front: %d busy = %b, right back: %d busy: %b",
                    leftFrontMotor.getCurrentPosition(), leftFrontMotor.isBusy(),
                    leftBackMotor.getCurrentPosition(), leftBackMotor.isBusy(),
                    rightFrontMotor.getCurrentPosition(), rightFrontMotor.isBusy(),
                    rightBackMotor.getCurrentPosition(), rightBackMotor.isBusy());
            //telemetry.addData("encoder-fwd", data);
            //telemetry.update();
            idle();
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        // ================== start of VISION ========================
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2, 16.0/9.0);
        }

        // ================== end of VISION =========================


        rightBackMotor = hardwareMap.get(DcMotor.class, "motor0");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "motor1");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "motor2");
        leftBackMotor = hardwareMap.get(DcMotor.class, "motor3");
        carouselMotor = hardwareMap.get(DcMotor.class, "motor5");
        armMotor = hardwareMap.get(DcMotor.class, "motor4");
        intakeMotor = hardwareMap.get(DcMotor.class, "motor6");



        // Creating my odometry object from the kinematics object. Here,
        // our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing forward.
//        MecanumDriveOdometry m_odometry = new MecanumDriveOdometry
//                (
//                        m_kinematics, getGyroHeading(),
//                        new Pose2d(5.0, 13.5, new Rotation2d()
//                        );

        waitForStart();

        // Start clicked
        long start = System.currentTimeMillis();

        // Slow move
        double power = 0.05;

        // Forward 100
        int distance = 100;
        setEncoderPositions(-distance, -distance, distance, distance);
        setPowers(-power, -power, power, power);
        waitForIdleMotor();

        detect();
        // Print out how much time we took.
        long end = System.currentTimeMillis();
        String output = String.format("objDetect = %s %d busy = %b, Time taken = %d ms ",
                detectDebug, leftBackMotor.getCurrentPosition(), leftBackMotor.isBusy(), end - start);
        telemetry.addData("DETECTION: ", output);
        telemetry.update();
        sleep(500);


        // going backwards
        distance = 100;
        setEncoderPositions(distance, distance, -distance, -distance);
        setPowers(power, power, -power, -power);
        waitForIdleMotor();
        sleep(500);

        //moving to the right 2x
        distance = 200;
        double powered = 0.05;

        // lf lb rf rb
        setEncoderPositions(-distance, distance, -distance, distance);
        setPowers(-powered, powered, -powered, powered);
        waitForIdleMotor();
        stopAllMotors();
        sleep(1000);

        // forward 3x
        distance = 600;
        setEncoderPositions(-distance, -distance, distance, distance);
        setPowers(-powered, -powered, powered, powered);
        waitForIdleMotor();
        sleep(500);

        if (objDetected == 1){
            distance = 600;
            setEncoderPositions(distance, -distance, distance, -distance);
            setPowers(powered, -powered, powered, -powered);
            waitForIdleMotor();

        } else if (objDetected == 2) {
            distance = 200;
            setEncoderPositions(distance, -distance, distance, -distance);
            setPowers(powered, -powered, powered, -powered);
            waitForIdleMotor();
        }
        stopAllMotors();
        sleep(30000);
    }
}
