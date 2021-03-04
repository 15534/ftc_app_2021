package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Wobble;

@Config
@TeleOp(name="Wobble Gripper Test")
public class ServoTest extends LinearOpMode {
    public static double pos = 0;
//    public Servo gripperServo;
//    public PwmControl gripperServoPWM;

    @Override
    public void runOpMode() throws InterruptedException {
//
//        gripperServo = hardwareMap.servo.get("wobble_gripper");
//        gripperServo.setDirection(Servo.Direction.FORWARD);
//        gripperServo.scaleRange(0, 1);
//        gripperServoPWM = (PwmControl) gripperServo;
//        PwmControl.PwmRange range = new PwmControl.PwmRange(800, 2200);
//        gripperServoPWM.setPwmRange(range);
//        gripperServoPWM.setPwmEnable();

        telemetry.addData("hello", "");
        telemetry.update();
        Wobble wobble = new Wobble(hardwareMap);
        waitForStart();
//        gripperServo.setPosition(0.5);
        while (opModeIsActive()) {
//            double pos = (1.0 + gamepad1.right_stick_y)/2.0;
            wobble.setGripper(pos);
            telemetry.addData("position", pos);
            telemetry.update();
        }

    }
}
