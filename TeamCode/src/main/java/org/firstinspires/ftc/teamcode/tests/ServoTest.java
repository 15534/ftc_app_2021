package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wobble;

@TeleOp(name="Wobble Gripper Test")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Wobble wobble = new Wobble(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double pos = (1.0 + gamepad1.right_stick_y)/2.0;
            wobble.setGripper(pos);
            telemetry.addData("position", pos);
            telemetry.update();
        }

    }
}
