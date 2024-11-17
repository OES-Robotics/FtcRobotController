package org.firstinspires.ftc.teamcode.listeners;

import org.firstinspires.ftc.teamcode.RobotState;

public class Wheels implements Listener {
    @Override
    public void onEvent(RobotState rs) {
        if (rs.gamepad1.x) {
            System.out.println("Fuck");
        }
    }
}