package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotState {
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;

    public DcMotor intakeMotor;

    public DcMotor leftSlideMotor;
    public DcMotor rightSlideMotor;

    public Servo intakeRotation;

    public RobotState(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
}
