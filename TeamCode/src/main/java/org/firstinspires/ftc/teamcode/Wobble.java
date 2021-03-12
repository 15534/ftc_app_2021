package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wobble {
    public Servo wobbleArm, wobbleGripper;
    public static double GRIPPER_RELEASE = 0.9;
    public static double GRIPPER_GRIPPED = 1;
    public static double ARM_UP = 0.15;
    public static double ARM_DOWN = 0.94;

    public boolean armUp = true;
    public boolean gripped = false;

    public Wobble (HardwareMap hardwareMap) {
        wobbleArm = hardwareMap.get(Servo.class, "wobble_arm");
        wobbleGripper = hardwareMap.get(Servo.class, "wobble_gripper");
    }

    public void setArm(double position) {
        wobbleArm.setPosition(position);
    }

    public void armUp () {
        wobbleArm.setPosition(ARM_UP);
        armUp = true;
    }

    public void armDown() {
        wobbleArm.setPosition(ARM_DOWN);
        armUp = false;
    }

    public void grip() {
        wobbleGripper.setPosition(GRIPPER_GRIPPED);
        gripped = true;
    }

    public void release() {
        wobbleGripper.setPosition(GRIPPER_RELEASE);
        gripped = false;
    }

    public void setGripper(double position) {
        wobbleGripper.setPosition(position);
    }
}
