package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    public static double DPAD_SPEED = 0.35;
    public static double BUMPER_ROTATION_SPEED = 0.4;
    public static double ROTATION_MULTIPLIER = 2.05;
    public static double STOPPER_POSITION = 0.395;
    public static double FLAP_POSITION = 0.1975;
    // parallel to shooter = 0.1975
    // max viable shooting angle = 0.14
    // shooting angle = 0.18
    // powershot angle = 0.2

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        CRServo transfer = hardwareMap.get(CRServo.class, "transfer");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        Servo flap = hardwareMap.get(Servo.class, "flap");
        Servo pusher = hardwareMap.get(Servo.class, "push");
        // 0.96 - resting position
        // 0.75 - pushed position

        GamepadEx gp1 = new GamepadEx(gamepad1);
        TriggerReader wobbleReader = new TriggerReader(gp1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        ButtonReader powershotButtonReader = new ButtonReader(gp1, GamepadKeys.Button.Y);
        ButtonReader pushButtonReader = new ButtonReader(gp1, GamepadKeys.Button.X);
        ButtonReader toggleIntake = new ButtonReader(gp1, GamepadKeys.Button.A);
        ButtonReader toggleShooter = new ButtonReader(gp1, GamepadKeys.Button.B);

        double lastPushTime = -1;
        boolean intakeActive = false;
        boolean shooterActive = false;
        boolean reverseIntake = false;
        double indexerPower = 0;
        double intakePower = 0;
        double transferPower = 0;

//        ModernRoboticsI2cRangeSensor rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_1");
        Wobble wobble = new Wobble(hardwareMap);
        wobble.armUp();

        Shooter shooter = new Shooter(hardwareMap);
        shooter.deactivate();

        flap.setPosition(FLAP_POSITION);
        stopper.setPosition(STOPPER_POSITION);

        waitForStart();
        while (!isStopRequested()) {
            pushButtonReader.readValue();
            powershotButtonReader.readValue();
            toggleIntake.readValue();
            wobbleReader.readValue();
            toggleShooter.readValue();

            Vector2d translation = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            double rotation = -ROTATION_MULTIPLIER*gamepad1.right_stick_x;

            // slow translation with dpad
            if (gamepad1.dpad_up) {
                translation = new Vector2d(DPAD_SPEED, 0);
            } else if (gamepad1.dpad_down) {
                translation = new Vector2d(-DPAD_SPEED, 0);
            } else if (gamepad1.dpad_left) {
                translation = new Vector2d(0, DPAD_SPEED);
            } else if (gamepad1.dpad_right) {
                translation = new Vector2d(0, -DPAD_SPEED);
            }

            // slow rotation with bumpers
            if (gamepad1.left_bumper) {
                rotation = BUMPER_ROTATION_SPEED;
            } else if (gamepad1.right_bumper) {
                rotation = -BUMPER_ROTATION_SPEED;
            }

            drive.setWeightedDrivePower(new Pose2d(translation, rotation));

            if (pushButtonReader.wasJustPressed()) {
                flap.setPosition(0.185); // shooting to top goal
                pusher.setPosition(PUSHED_POSITION);
                lastPushTime = runtime.seconds();
            } else if (powershotButtonReader.wasJustPressed()) {
                flap.setPosition(0.205); // shooting powershot (lowered flap due to height decrease)
                pusher.setPosition(PUSHED_POSITION);
                lastPushTime = runtime.seconds();
            }

            if (runtime.seconds() - lastPushTime > 0.2) {
                pusher.setPosition(RELEASED_POSITION);
                lastPushTime = 0;
            }

            if (toggleIntake.wasJustPressed()) {
                intakeActive = !intakeActive;
            }

            if (toggleShooter.wasJustPressed()) {
                shooterActive = !shooterActive;
                if (shooterActive) {
                    transferPower = 1;
                    shooter.activate();
                    stopper.setPosition(0.01);
                } else {
                    transferPower = 0;
                    shooter.deactivate();
                    stopper.setPosition(STOPPER_POSITION);
                }
            }

            if (intakeActive) {
                intakePower = 1;
            } else {
                intakePower = 0;
            }

            if (intakeActive || shooterActive) {
                indexerPower = INDEXER_POWER;
            } else {
                indexerPower = 0;
            }

            if (toggleIntake.isDown() && toggleShooter.isDown()) {
                intake.setPower(-0.25);
                indexer.setPower(-0.65);
                transfer.setPower(-0.5);
                shooter.deactivate();
            } else {
                    indexer.setPower(indexerPower);
                    intake.setPower(intakePower);
                    transfer.setPower(transferPower);
            }

            telemetry.addData("Right trigger down", wobbleReader.isDown());

            if (wobbleReader.wasJustPressed()) {
                telemetry.addData("WOBBLE PRESSED", 1);
                if (wobble.armUp) {
                    wobble.armDown();
                } else {
                    wobble.armUp();
                }
            }
            if (wobbleReader.wasJustReleased()) {
                telemetry.addData("WOBBLE RELEASED", 1);
            }
            wobble.setGripper(GRIPPER_RELEASE);

//            telemetry.addData("wobble gripper", "%.2f", wobble.getGripper());
            telemetry.addData("shooter velocity", "%.2f", shooter.velocity());
//            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
//            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
