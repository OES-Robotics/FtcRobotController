package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.InternalState;

public class Intake implements Component {
    public Intake() {}

    @Override
    public void update(InternalState rs) {
        final int intakeSlideOut = 938;
        final double intakeOut = 0.75;

        if (rs.intakeSlide.getCurrentPosition() > (intakeSlideOut - 700)) {
            if (!rs.gamepad2.x) {
                rs.intake.setDirection(DcMotor.Direction.REVERSE);
                rs.intake.setPower(0.5);
            }
            if (!rs.gamepad2.b)
                rs.intakeFlip.setPosition(intakeOut);

        } else {
            rs.intakeFlip.setPosition(0);
            rs.intake.setPower(0);
        }
        if (rs.gamepad2.x) {
            rs.intake.setDirection(DcMotor.Direction.FORWARD);
            rs.intake.setPower(0.5);
        }
        if (rs.gamepad2.b)
            rs.intakeFlip.setPosition(intakeOut);
        else
            rs.intakeFlip.setPosition(0);

    }
}
