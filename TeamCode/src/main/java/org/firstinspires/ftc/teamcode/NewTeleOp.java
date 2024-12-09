package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.components.Intake;

import org.firstinspires.ftc.teamcode.components.Wheels;

import java.util.List;
import java.util.function.BiFunction;

@TeleOp(name = "NewTeleOp", group = "Linear OpMode")
public class NewTeleOp extends LinearOpMode {
    private final RobotState rs;

    private final List<Component> components;

    public NewTeleOp() {
        BiFunction<DcMotor, DcMotorSimple.Direction, Void> init_motor = (motor, direction) -> {
            motor.setDirection(direction);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return null;
        };

        rs = new RobotState(gamepad1, gamepad2) {{
            // Wheel motors
            leftFront = hardwareMap.dcMotor.get("left_front_drive");
            leftBack = hardwareMap.dcMotor.get("left_back_drive");
            rightFront = hardwareMap.dcMotor.get("right_front_drive");
            rightBack = hardwareMap.dcMotor.get("right_back_drive");

            // Intake system
            intake = hardwareMap.dcMotor.get("intake_motor");
            intakeSlide = hardwareMap.dcMotor.get("intake_slide");
            intakeFlip = hardwareMap.servo.get("intake_flip");
            init_motor.apply(intakeSlide, DcMotorSimple.Direction.FORWARD);

            leftSlide = hardwareMap.dcMotor.get("left_slide");
            rightSlide = hardwareMap.dcMotor.get("right_slide");
            init_motor.apply(leftSlide, DcMotorSimple.Direction.REVERSE);
            init_motor.apply(rightSlide, DcMotorSimple.Direction.FORWARD);
        }};

        components = List.of(
                new Wheels(),
                new Intake()
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        // Wait for click of start button
        waitForStart();

        // Main loop
        while (opModeIsActive())
            components.forEach(l -> l.update(rs));  // Call update for each component
    }
}
