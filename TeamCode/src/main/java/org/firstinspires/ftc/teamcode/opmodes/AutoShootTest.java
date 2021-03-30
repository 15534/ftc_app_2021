package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        Servo flap = hardwareMap.get(Servo.class, "flap");
        CRServo transfer = hardwareMap.get(CRServo.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        flap.setPosition(0.1845);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();
        runtime.reset();
        shooter.activate();
        transfer.setPower(1);

        double transferTime = 0;
        double shootTime = 0;

        //loop
        while (opModeIsActive()) {
            double elapsed = runtime.seconds();
            if (shooter.readyForTransfer() && (transferTime == 0)) {
                indexer.setPower(1);
                transferTime = runtime.seconds();
            }


            if (shooter.ready() && (elapsed - transferTime > 0.5) && (shootTime == 0)) {
                shootTime = elapsed;
                shooter.push();
            }

            if ((shootTime != 0) && (elapsed - shootTime > 0.2)) {
                shooter.release();
            }

            if ((shootTime != 0) && (elapsed - shootTime > 0.4)) {
                shooter.push();
            }

            if ((shootTime != 0) && (elapsed - shootTime > 0.6)) {
                shooter.release();
            }

            if ((shootTime != 0) && (elapsed - shootTime > 0.8)) {
                shooter.push();
            }

            if ((shootTime != 0) && (elapsed - shootTime > 1)) {
                shooter.release();
            }

            if (elapsed - shootTime > 2) break;

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("velocity", shooter.velocity());
            packet.put("power", shooter.leftShoot.getPower());
            packet.put("elapsed", elapsed);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
