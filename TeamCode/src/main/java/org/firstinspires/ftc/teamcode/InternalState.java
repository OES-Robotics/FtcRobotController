package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class InternalState {
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor intake;
    public DcMotor intakeSlide;
    public Servo intakeFlip;

    public DcMotor leftSlide;
    public DcMotor rightSlide;

    public InternalState(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
}
