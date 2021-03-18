package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Shooter;
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
    public static double BUMPER_ROTATION_SPEED = 0.35;
    public static double ROTATION_MULTIPLIER = 2.05;
    public static double FLAP_POSITION = 0.1975;
    // parallel to shooter = 0.1975
    // max viable shooting angle = 0.14
    // shooting angle = 0.18
    // powershot angle = 0.2

    private ElapsedTime runtime = new ElapsedTime();

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        CRServo transfer = hardwareMap.get(CRServo.class, "transfer");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo flap = hardwareMap.get(Servo.class, "flap");
        Servo pusher = hardwareMap.get(Servo.class, "push");
        // 0.96 - resting position
        // 0.75 - pushed position

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        ButtonReader toggleShooter = new ButtonReader(gp2, GamepadKeys.Button.A);
        ButtonReader powershotButtonReader = new ButtonReader(gp2, GamepadKeys.Button.Y);
        ButtonReader pushButtonReader = new ButtonReader(gp2, GamepadKeys.Button.X);
        ButtonReader reverseAll = new ButtonReader(gp2, GamepadKeys.Button.B);
        ButtonReader goToShootPosition = new ButtonReader(gp1, GamepadKeys.Button.A);

        double lastPushTime = -1;
        boolean intakeActive = false;
        boolean shooterActive = false;
        double indexerPower = 0;
        double intakePower = 0;
        double transferPower = 0;

//        ModernRoboticsI2cRangeSensor rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_1");
        Wobble wobble = new Wobble(hardwareMap);
        wobble.armUp();

        Shooter shooter = new Shooter(hardwareMap);
        shooter.deactivate();

        flap.setPosition(FLAP_POSITION);

        waitForStart();
        while (!isStopRequested()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            pushButtonReader.readValue();
            powershotButtonReader.readValue();
            reverseAll.readValue();
            toggleShooter.readValue();
            goToShootPosition.readValue();

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

            switch(currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(new Pose2d(translation, rotation));

                    if (gamepad1.a) {
                        Trajectory goToShoot = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(0,0,0), 0)
                                .build();
                        drive.followTrajectoryAsync(goToShoot);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    if (gamepad1.y) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }

            if (gamepad1.x) {
                drive.setPoseEstimate(new Pose2d(0, 0, 0));
            }

            if (pushButtonReader.wasJustPressed()) {
                flap.setPosition(0.1845); // shooting to top goal
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

            if (toggleShooter.wasJustPressed()) {
                shooterActive = !shooterActive;
                if (shooterActive) {
                    shooter.activate();
                } else {
                    shooter.deactivate();
                }
            }

            intakeActive = gamepad2.right_trigger > 0.05;

            if (intakeActive) {
                intakePower = 1;
            } else {
                intakePower = 0;
            }

            if (intakeActive || (shooterActive && shooter.ready())) {
                indexerPower = INDEXER_POWER;
            } else {
                indexerPower = 0;
            }

            if (shooterActive && shooter.ready()) {
                transferPower = 1;
            } else {
                transferPower = 0;
            }

            if (reverseAll.isDown()) {
                shooter.deactivate();
                intake.setPower(-0.25);
                indexer.setPower(-0.65);
                transfer.setPower(-0.5);
            } else {
                indexer.setPower(indexerPower);
                intake.setPower(intakePower);
                transfer.setPower(transferPower);
            }

            if (gamepad2.dpad_down) {
                wobble.armDown();
            }
            if (gamepad2.dpad_up) {
                wobble.armUp();
            }
            if (gamepad2.dpad_left) {
                wobble.release();
            }
            if (gamepad2.dpad_right) {
                wobble.grip();
            }

            if (gamepad2.left_bumper) {
                drive.turn(Math.toRadians(5));
            } else if (gamepad2.right_bumper) {
                drive.turn(Math.toRadians(-5));
            }

//            shooterWasActive = gamepad1.left_trigger > 0.05;

            telemetry.addData("shooter velocity", "%.2f", shooter.velocity());
//            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
//            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
