package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Wobble;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Wobble.GRIPPER_RELEASE;

@TeleOp(name="Tele-op")
@Config
public class MainTeleOp extends LinearOpMode {
    public static double INDEXER_POWER = 1;
    public static double PUSHED_POSITION = 0.75;
    public static double RELEASED_POSITION = 0.96;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo pusher = hardwareMap.get(Servo.class, "push");
        // 0.96 - resting position
        // 0.75 - pushed position

        GamepadEx gp1 = new GamepadEx(gamepad1);
        ButtonReader wobbleButtonReader = new ButtonReader(gp1, GamepadKeys.Button.A);
        ButtonReader pushButtonReader = new ButtonReader(gp1, GamepadKeys.Button.X);
        ButtonReader toggleIndexer = new ButtonReader(gp1, GamepadKeys.Button.Y);
        double lastPushTime = -1;
        boolean indexerActive = false;

//        ModernRoboticsI2cRangeSensor rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_1");
        Wobble wobble = new Wobble(hardwareMap);
        wobble.armUp();

        waitForStart();
        while (!isStopRequested()) {
            pushButtonReader.readValue();
            toggleIndexer.readValue();
            wobbleButtonReader.readValue();
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

            if (pushButtonReader.wasJustPressed()) {
                telemetry.addData("pusher", "pushed position");
                pusher.setPosition(PUSHED_POSITION);
                lastPushTime = runtime.seconds();
            }

            if (runtime.seconds() - lastPushTime > 0.2) {
                telemetry.addData("pusher", "released position");
                pusher.setPosition(RELEASED_POSITION);
                lastPushTime = 0;
            }

            if (toggleIndexer.wasJustPressed()) {
                indexerActive = !indexerActive;
            }

            if (indexerActive) {
                indexer.setPower(INDEXER_POWER);
            } else {
                indexer.setPower(0);
            }

            if (wobbleButtonReader.wasJustPressed()) {
                if (wobble.armUp) {
                    wobble.armDown();
                } else {
                    wobble.armUp();
                }
            }
            wobble.setGripper(GRIPPER_RELEASE);

            telemetry.addData("wobble gripper", "%.2f", wobble.getGripper());
//            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
//            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
