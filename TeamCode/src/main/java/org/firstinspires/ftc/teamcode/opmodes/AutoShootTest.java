package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutoShootTest")

public class AutoShootTest extends LinearOpMode {

    double time = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    private FtcDashboard dashboard;

    enum State {
        IDLE
    }


    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //constants
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        Servo flap = hardwareMap.get(Servo.class, "flap");
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flap.setPosition(0.1845);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();
        runtime.reset();
        shooter.activate();
        shooter.allow();
        indexer.setPower(1);
        intake.setPower(1);
        double shootTime = 0.5;

        //loop
        while (opModeIsActive()) {
            double elapsed = runtime.seconds();

            if (elapsed < shootTime) {

            } else if (elapsed < shootTime + 0.2) {
                shooter.push();
            } else if (elapsed < shootTime + 0.6) {
                shooter.release();
            } else if (elapsed < shootTime + 0.8) {
                shooter.push();
            } else if (elapsed < shootTime + 1) {
                shooter.release();
            } else if (elapsed < shootTime + 1.2) {
                shooter.push();
            } else if (elapsed < shootTime + 1.4) {
                shooter.release();
            } else if (elapsed < shootTime + 1.6) {
                shooter.push();
            } else if (elapsed < shootTime + 1.8) {
                shooter.release();
            } else if (elapsed < shootTime + 2) {
                shooter.push();
            } else if (elapsed < shootTime + 3) {
                shooter.release();
            }
//            else if (elapsed < shootTime + 0.6) {
//                shooter.push();
//            } else if (elapsed < shootTime + 0.8) {
//                shooter.release();
//            } else if (elapsed < shootTime + 1.0) {
//                shooter.push();
//            } else if (elapsed < shootTime + 1.2) {
//                shooter.release();
//            }
            else {
                shooter.deactivate();
                shooter.block();
                intake.setPower(0);
                indexer.setPower(0);
            }

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("velocity", shooter.velocity());
            packet.put("power", shooter.leftShoot.getPower());
            packet.put("elapsed", elapsed);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
