package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-desig                        c`                                                                                                                                                                                                                                                                                                 n/drivetrains/holonomic.html
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

enum MotorType {
    LEFT_FRONT("left_front_drive"),
    LEFT_BACK("left_back_drive"),
    RIGHT_FRONT("right_front_drive"),
    RIGHT_BACK("right_back_drive"),
    LEFT_SLIDE("left_slide"),
    RIGHT_SLIDE("right_slide");

    MotorType(String name) {
    }

    @Override
    public String toString() {
        return this.name();
    }
}

@TeleOp(name = "MainTeleOp", group = "Linear OpMode")
//Disabled
public class MainTeleOp extends LinearOpMode {


    private final Map<MotorType, DcMotor> motors = new HashMap<>();

    private ElapsedTime runtime = new ElapsedTime();
    private CRServo intake = null;
    private CRServo intake_winch = null;
    private Servo intake_rotation = null;

    private int slideOut = 2200;
    private int slideIn = 0;

    private void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        Map<MotorType, Double> powers = new HashMap<MotorType, Double>() {{
            put(MotorType.LEFT_FRONT, y + x + yaw);
            put(MotorType.RIGHT_FRONT, y - x - yaw);
            put(MotorType.LEFT_BACK, y - x + yaw);
            put(MotorType.RIGHT_BACK, y + x - yaw);
        }};

        // Normalize wheel powers to be less than 1.0
        double max = powers
                .values()
                .stream()
                .map(Math::abs)
                .max(Double::compare)
                .orElse(1.0);
        if (max > 1.0)
            powers.keySet().forEach(motorType -> powers.put(motorType, powers.get(motorType) / max));

        // Set motor powers
        double power_module = 0.5f;
        for (MotorType motorType : MotorType.values())
            motors.get(motorType).setPower(powers.get(motorType) * power_module);

        // log data
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor Powers", powers);
    }

    @Override
    public void runOpMode() {
        for (MotorType motorType : MotorType.values())
            motors.put(motorType, hardwareMap.get(DcMotor.class, motorType.name()));

        intake = hardwareMap.get(CRServo.class, "intake");
        intake_winch = hardwareMap.get(CRServo.class, "intake_winch");
        intake_rotation = hardwareMap.get(Servo.class, "intake_rotation");

        motors.get(MotorType.LEFT_FRONT).setDirection(DcMotor.Direction.REVERSE);
        motors.get(MotorType.LEFT_BACK).setDirection(DcMotor.Direction.REVERSE);
        motors.get(MotorType.RIGHT_FRONT).setDirection(DcMotor.Direction.FORWARD);
        motors.get(MotorType.RIGHT_BACK).setDirection(DcMotor.Direction.FORWARD);

        DcMotor leftSlideDrive = motors.get(MotorType.LEFT_SLIDE);
        assert leftSlideDrive != null;
        leftSlideDrive.setDirection(DcMotor.Direction.FORWARD);
        leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideDrive.setTargetPosition(0);
        leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DcMotor rightSlideDrive = motors.get(MotorType.RIGHT_SLIDE);
        assert rightSlideDrive != null;
        rightSlideDrive.setDirection(DcMotor.Direction.REVERSE);
        rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideDrive.setTargetPosition(0);
        rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double rotation = 0;
        boolean slide_active = false;
        boolean button_pressed = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            moveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad2.x)
                rotation = Math.min(rotation + 1, 270);
            if (gamepad2.y)
                rotation = Math.max(rotation - 1, 0);

            intake_rotation.setPosition(rotation / 270.0);
            telemetry.addData("Servo_rotation", rotation);

            runGamepadInput(gamepad1, intake);
            runGamepadInput(gamepad2, intake_winch);

            if (gamepad1.x && !button_pressed)
                button_pressed = slide_active = true;
            else if (button_pressed)
                button_pressed = false;


            if (slide_active) {
                leftSlideDrive.setTargetPosition(slideOut);
                rightSlideDrive.setTargetPosition(slideOut);

                leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftSlideDrive.setPower(0.75);
                rightSlideDrive.setPower(0.75);

                telemetry.addData("Status", "Moving to position: " + slideOut);
            } else {
                leftSlideDrive.setTargetPosition(slideIn);
                rightSlideDrive.setTargetPosition(slideIn);

                leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftSlideDrive.setPower(0.5);
                rightSlideDrive.setPower(0.5);

                telemetry.addData("Status", "Moving to position: " + slideIn);
            }

            // Once target is reached, motor will automatically stop.
            if (!leftSlideDrive.isBusy()) {
                leftSlideDrive.setPower(0);
                telemetry.addData("Status", "left reached");
            }

            if (!rightSlideDrive.isBusy()) {
                rightSlideDrive.setPower(0);
                telemetry.addData("Status", "right reached");
            }

            telemetry.addData("Status", "Slide at %7d", leftSlideDrive.getCurrentPosition());
            telemetry.addData("Status", "Slide at %7d", rightSlideDrive.getCurrentPosition());
            telemetry.update();
        }
    }

    private void runGamepadInput(Gamepad gamepad2, CRServo intake) {
        if (gamepad2.a) {
            intake.setPower(1);
            telemetry.addData("Status", "servo move?");
        } else if (gamepad2.b) {
            intake.setPower(-1);
            telemetry.addData("Status", "servo other move");
        } else {
            intake.setPower(0);
            telemetry.addData("Status", "servo no move");
        }
    }
}
