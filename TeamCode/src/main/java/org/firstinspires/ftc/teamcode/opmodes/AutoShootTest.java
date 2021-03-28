package org.firstinspires.ftc.teamcode.opmodes;

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

        flap.setPosition(0.1845);
        waitForStart();
        runtime.reset();
        double pushTime = -1;
        int ringsShot = 0;
        shooter.activate();
        boolean isPushed = false;

        //loop
        while (opModeIsActive()) {
            double elapsed = runtime.seconds();
            if (shooter.readyForTransfer()) {
                transfer.setPower(1);
            } else {
                transfer.setPower(0);
            }
            if (shooter.ready() && !isPushed) {
                shooter.push();
                ringsShot++;
                pushTime = elapsed;
                isPushed = true;
            }
            if (isPushed) {
                if (elapsed - pushTime > 0.2) {
                    shooter.release();
                }
                if (elapsed - pushTime > 0.4) {
                    isPushed = false;
                }
            }

            if (ringsShot == 2) {
                shooter.deactivate();
            }

            telemetry.addData("velocity", shooter.velocity());
            telemetry.addData("elapsed", elapsed);
            telemetry.update();
        }
    }
}
