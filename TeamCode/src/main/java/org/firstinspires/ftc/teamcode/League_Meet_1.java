package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="League_Meet_1", group="Linear OpMode")
//Disabled
public class League_Meet_1 extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftUpSlideDrive = null;
    private DcMotor rightUpSlideDrive = null;

    private DcMotor leftOutSlideDrive = null;

    private DcMotor intake = null;

    private ElapsedTime runtime = new ElapsedTime();
    private Servo intakeFlip = null;

    private int slideOut = 976 / 120 * 537 - 100;
    // 537.7 pulses/output shaft rotation
    // 38.2 mm pitch diameter (120.0 mm circumference)
    // 976 mm stroke length
    private int slideIn = 0;
    private int intakeSlideOut = 489 / 120 * 537 - 1250;
    // 489.6 mm stroke length
    private double intakeOut = 0.75;
    private double intakeIn = 0;


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double tmp = x;
        x = y;
        y = yaw;
        yaw = tmp;
        double leftFrontPower    = y + x + yaw;
        double rightFrontPower   = y - x - yaw;
        double leftBackPower     = y - x + yaw;
        double rightBackPower    = y + x - yaw;
        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        intakeFlip = hardwareMap.get(Servo.class, "intake_flip");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // vertical slides
        leftUpSlideDrive = hardwareMap.get(DcMotor.class, "left_slide");
        rightUpSlideDrive = hardwareMap.get(DcMotor.class, "right_slide");

        leftUpSlideDrive.setDirection(DcMotor.Direction.REVERSE);
        leftUpSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftUpSlideDrive.setTargetPosition(0);
        leftUpSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightUpSlideDrive.setDirection(DcMotor.Direction.FORWARD);
        rightUpSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightUpSlideDrive.setTargetPosition(0);
        rightUpSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int leftUpSlidePosition = leftUpSlideDrive.getCurrentPosition();
        int rightUpSlidePosition = rightUpSlideDrive.getCurrentPosition();

        // horizontal slides
        leftOutSlideDrive = hardwareMap.get(DcMotor.class, "intake_slide");
        leftOutSlideDrive.setDirection(DcMotor.Direction.FORWARD);
        leftOutSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOutSlideDrive.setTargetPosition(0);
        leftOutSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            moveRobot(-gamepad1.right_stick_x, -gamepad1.left_stick_x, -gamepad1.left_stick_y);

            if (leftOutSlideDrive.getCurrentPosition() > (intakeSlideOut - 700)) {
                if (!gamepad2.x) {
                    intake.setDirection(DcMotor.Direction.REVERSE);
                    intake.setPower(0.5);
                }
                if (!gamepad2.b) {
                    intakeFlip.setPosition(intakeOut);
                }
            }
            else {
                intakeFlip.setPosition(intakeIn);
                intake.setPower(0);
            }
            if (gamepad2.x) {
                intake.setDirection(DcMotor.Direction.FORWARD);
                intake.setPower(0.5);
            }
            if (gamepad2.b) {
                intakeFlip.setPosition(intakeOut);
                //intakeFlip.setMode(Servo.RunMode.RUN_TO_POSITION);
                //intakeFlip.setPower(0.5);
            }


            else {
                intakeFlip.setPosition(intakeIn);
            }
            /*
            if (gamepad2.a) {
                intake.setDirection(DcMotor.Direction.FORWARD);
                intake.setPower(0.5);
            }
            */


            if (gamepad1.left_trigger >= 0.75) {
                leftUpSlideDrive.setTargetPosition(slideOut);
                rightUpSlideDrive.setTargetPosition(slideOut);

                leftUpSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightUpSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftUpSlideDrive.setPower(1);
                rightUpSlideDrive.setPower(1);
            }

            else {
                leftUpSlideDrive.setTargetPosition(slideIn);
                rightUpSlideDrive.setTargetPosition(slideIn);

                leftUpSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightUpSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftUpSlideDrive.setPower(1);
                rightUpSlideDrive.setPower(1);
            }

            if (gamepad2.left_bumper) {
                leftOutSlideDrive.setTargetPosition(intakeSlideOut);
                leftOutSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftOutSlideDrive.setPower(0.75);
            }


            else {
                leftOutSlideDrive.setTargetPosition(0);
                leftOutSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftOutSlideDrive.setPower(0.75);
            }

            // Once target is reached, motor will automatically stop.
            if (!leftUpSlideDrive.isBusy()) {
                leftUpSlideDrive.setPower(0);
            }

            if (!rightUpSlideDrive.isBusy()) {
                rightUpSlideDrive.setPower(0);
            }
        }
    }
/*
    @Override
    public void stop() {
        leftUpSlideDrive.setPower(0);
        rightUpSlideDrive.setPower(0);

        leftOutSlideDrive.setPower(0);
        rightOutSlideDrive.setPower(0);

        intake.setPower(0);
    }
    */
}