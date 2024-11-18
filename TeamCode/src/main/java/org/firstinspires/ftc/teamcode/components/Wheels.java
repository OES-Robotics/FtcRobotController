package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotState;

public class Wheels implements Component {
    @Override
    public void update(RobotState rs) {
        final double x = -rs.gamepad1.left_stick_x;
        final double y = -rs.gamepad1.left_stick_y;
        final double yaw = -rs.gamepad1.right_stick_x;

        move(x, y, yaw, rs.leftFront, rs.rightFront, rs.leftBack, rs.rightBack);
    }

    private void move(final double x,
                      final double y,
                      final double yaw,
                      final DcMotor leftFront,
                      final DcMotor rightFront,
                      final DcMotor leftBack,
                      final DcMotor rightBack) {
        double leftFrontPower    = y + x + yaw;
        double rightFrontPower   = y - x - yaw;
        double leftBackPower     = y - x + yaw;
        double rightBackPower    = y + x - yaw;

        // Normalize wheel powers to be less than 1.0
        final double max;
        max = Math.max(Math.abs(leftFrontPower),
        Math.max(Math.abs(rightFrontPower),
        Math.max(Math.abs(leftBackPower),
                Math.abs(rightBackPower))));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
}