package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.listeners.Listener;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "NewTeleOp", group = "Linear OpMode")
public class NewTeleOp extends LinearOpMode {
    private final RobotState rs;

    private final List<Listener> listeners;

    public NewTeleOp() {
        rs = new RobotState(gamepad1, gamepad2) {{
            // wheel motors
            leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
            leftBackMotor = hardwareMap.dcMotor.get("left_back_drive");
            rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
            rightBackMotor = hardwareMap.dcMotor.get("right_back_drive");
            // intake motor
            intakeMotor = hardwareMap.dcMotor.get("intake_motor");
            // slide motors
            leftSlideMotor = hardwareMap.dcMotor.get("left_slide");
            rightSlideMotor = hardwareMap.dcMotor.get("right_slide");
            // intake rotation servo
            intake_rotation = hardwareMap.servo.get("intake_rotation");
        }};

        listeners = new ArrayList<>();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    void addEventListener(Listener l) {
        listeners.add(l);
    }

    void addEventListeners(Listener... listeners) {
        for (Listener l : listeners) addEventListener(l);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for click of start button
        waitForStart();

        // Main loop
        while (opModeIsActive())
            listeners.forEach(l -> l.onEvent(rs));  // Call onEvent for each listener
    }
}
