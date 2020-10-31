package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@Autonomous(name="Shooter Tuner")
public class ShooterPIDTuner extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private DcMotorEx leftShoot, rightShoot;

    // TODO tune this better - reduce oscillations
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(6700, 0, 0, 0);

    private double lastKp = MOTOR_VELO_PID.p;
    private double lastKi = MOTOR_VELO_PID.i;
    private double lastKd = MOTOR_VELO_PID.d;
    private double lastKf = MOTOR_VELO_PID.f;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        leftShoot = hardwareMap.get(DcMotorEx.class, "left");
        rightShoot = hardwareMap.get(DcMotorEx.class, "right");
        leftShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShoot.setDirection(DcMotor.Direction.REVERSE);
        rightShoot.setDirection(DcMotor.Direction.REVERSE);
        leftShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        rightShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            double targetPower = 1140;
            leftShoot.setVelocity(targetPower);
            rightShoot.setVelocity(targetPower);

            double leftVelocity = leftShoot.getVelocity();
            double rightVelocity = leftShoot.getVelocity();

            // update telemetry
            telemetry.addData("targetVelocity", targetPower);
            telemetry.addData("leftVelocity", leftVelocity);
            telemetry.addData("rightVelocity", rightVelocity);
            telemetry.addData("leftError", targetPower - leftVelocity);
            telemetry.addData("rightError", targetPower - rightVelocity);

            if (lastKp != MOTOR_VELO_PID.p || lastKd != MOTOR_VELO_PID.d
                    || lastKi != MOTOR_VELO_PID.i || lastKf != MOTOR_VELO_PID.f) {
                leftShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            telemetry.update();
        }
    }
}
