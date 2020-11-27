package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wobble {
    public Servo wobbleArm, wobbleGripper;
    public PwmControl wobbleGripperPwm;

    public static double GRIPPER_RELEASE = 0.5;
    public static double ARM_UP = 0.15;
    public static double ARM_DOWN = 0.94;

    public boolean armUp = true;

    public Wobble (HardwareMap hardwareMap) {
        wobbleArm = hardwareMap.get(Servo.class, "wobble_arm");
        wobbleGripper = hardwareMap.get(Servo.class, "wobble_gripper");
        wobbleGripperPwm = (PwmControl) wobbleGripper;

        PwmControl.PwmRange range = new PwmControl.PwmRange(800, 2200);
        wobbleGripperPwm.setPwmRange(range);
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

    public void setGripper(double position) {
        wobbleGripper.setPosition(position);
    }
    public double getArm() {
        return wobbleArm.getPosition();
    }

    public double getGripper() {
        return wobbleGripper.getPosition();
    }
}
