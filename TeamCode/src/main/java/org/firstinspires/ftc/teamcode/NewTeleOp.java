package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.Component;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "NewTeleOp", group = "Linear OpMode")
public class NewTeleOp extends LinearOpMode {
    private final RobotState rs;

    private final List<Component> components;

    public NewTeleOp() {
        rs = new RobotState(gamepad1, gamepad2) {{
            // Wheel motors
            leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
            leftBackMotor = hardwareMap.dcMotor.get("left_back_drive");
            rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
            rightBackMotor = hardwareMap.dcMotor.get("right_back_drive");

            // Intake system
            intakeMotor = hardwareMap.dcMotor.get("intake_motor");
            intakeRotation = hardwareMap.servo.get("intake_rotation");

            leftSlideMotor = hardwareMap.dcMotor.get("left_slide");
            rightSlideMotor = hardwareMap.dcMotor.get("right_slide");
        }};

        components = new ArrayList<>();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    void addComponent(Component l) {
        components.add(l);
    }

    void addEventListeners(Component... components) {
        for (Component l : components)
            addComponent(l);
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
