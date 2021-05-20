package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wobble {
    public Servo wobbleArm, wobbleGripper;
    public PwmControl wobbleGripperPwm;
    public static double GRIPPER_RELEASE = 0.4;
    public static double GRIPPER_GRIPPED = 0;
    public static double GRIPPER_LOOSEN = 0.18;
    public static double ARM_UP = 0;
    public static double ARM_DOWN = 0.92;  // old: 0.96
    public static double ARM_LOW = 0.8;
    public static double ARM_MIDDLE = 0.3;

    public boolean armUp = true;
    public boolean gripped = false;

    public Wobble (HardwareMap hardwareMap) {
        wobbleArm = hardwareMap.get(Servo.class, "wobble_arm");
        wobbleGripper = hardwareMap.get(Servo.class, "wobble_gripper");
        wobbleGripper.scaleRange(0, 1);
        wobbleGripperPwm = (PwmControl) wobbleGripper;
        wobbleGripperPwm.setPwmRange(new PwmControl.PwmRange(800, 2200));
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

    public void armMiddle() {
        wobbleArm.setPosition(ARM_MIDDLE);
        armUp = true;
    }

    public void armLow() {
        wobbleArm.setPosition(ARM_LOW);
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

    public void loosen(){
        wobbleGripper.setPosition(GRIPPER_LOOSEN);
        gripped = true;
    }

    public void toggleGrip() {
        if (gripped) release();
        else grip();
    }

    public void setGripper(double position) {
        wobbleGripper.setPosition(position);
    }
}
