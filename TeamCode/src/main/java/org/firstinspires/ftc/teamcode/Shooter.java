package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter {
    DcMotorEx leftShoot, rightShoot;
    Servo pusher;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(500, 0, 0, 0);
    public static double SHOOTER_SPEED = 1075;
    public static double ACTIVATION_SPEED = 800;
    public static double IDEAL_SHOOT_SPEED = 1050;
    public static double PUSHED_POSITION = 0.56;
    public static double RELEASED_POSITION = 0.81;

    public Shooter (HardwareMap hardwareMap) {
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
        pusher = hardwareMap.get(Servo.class, "push");
    }

    public void activate() {
        leftShoot.setVelocity(SHOOTER_SPEED);
        rightShoot.setVelocity(SHOOTER_SPEED);
    }

    public void deactivate() {
        leftShoot.setVelocity(0);
        rightShoot.setVelocity(0);
    }

    public double velocity() {
        return leftShoot.getVelocity();
    }
    public boolean readyForTransfer() {
        return leftShoot.getVelocity() > ACTIVATION_SPEED;
    }
    public boolean ready() {
        return leftShoot.getVelocity() >= IDEAL_SHOOT_SPEED;
    }
    public void push() {
        pusher.setPosition(PUSHED_POSITION);
    }
    public void release() {
        pusher.setPosition(RELEASED_POSITION);
    }
}
