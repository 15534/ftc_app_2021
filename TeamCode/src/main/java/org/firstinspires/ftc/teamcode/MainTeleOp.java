package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Tele-op")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        while (!isStopRequested()) {
            if (Math.abs(gamepad1.left_stick_x) > 0.05 || (Math.abs(gamepad1.left_stick_y) > 0.05)
                    || (Math.abs(gamepad1.right_stick_x) > 0.05)) {
                if (Math.abs(gamepad1.right_stick_x) < 0.05) {
                    // tank driving - left joystick controls left wheels,
                    // right joystick controls right wheels
                    drive.setDriveSignal(new DriveSignal(
                            new Pose2d(-300*gamepad1.left_stick_y,-300*gamepad1.left_stick_x, 0)));
                } else {
                    // rotate by moving the right stick horizontally
                    drive.setDriveSignal(new DriveSignal(
                            new Pose2d(-100*gamepad1.left_stick_y,-100*gamepad1.left_stick_x, -3*gamepad1.right_stick_x)));
                }
            } else {
                drive.setDriveSignal(new DriveSignal(
                        new Pose2d(0, 0,0)));
            }
        }
    }
}
