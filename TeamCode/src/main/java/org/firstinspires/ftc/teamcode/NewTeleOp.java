package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.components.Wheels;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "NewTeleOp", group = "Linear OpMode")
public class NewTeleOp extends LinearOpMode {
    private final RobotState rs;

    private final List<Component> components;

    public NewTeleOp() {
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

            leftSlide = hardwareMap.dcMotor.get("left_slide");
            rightSlide = hardwareMap.dcMotor.get("right_slide");
        }};

        components = new ArrayList<>();
        this.addComponents(new Wheels());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    void addComponent(Component l) {
        components.add(l);
    }

    void addComponents(Component... components) {
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
