package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RedAuto")
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startingPosition = new Pose2d(-60, -48, Math.toRadians(0));
        Trajectory trajectoryForward = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(-36,-48))
                .splineToConstantHeading(new Vector2d(12, -12), Math.toRadians(0))
                .build();

        drive.setPoseEstimate(startingPosition);
        waitForStart();

        drive.followTrajectoryAsync(trajectoryForward);

        while (opModeIsActive()) {
            drive.update();
        }
    }
}
