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
        //starting position for robot - halfway across first tile
        Pose2d startingPosition = new Pose2d(-60, -60, Math.toRadians(0)); //maximum starting position

        //trajectory to go to tile a to drop off wobble goal
        Trajectory trajectoryForwardA = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(0,-60))
                .build();

        //trajectory to go to tile b to drop off wobble goal
        Trajectory trajectoryForwardB = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(24, -36)) //maybe need to change lineTo for this
                .build();

        //trajectory to go to tile b to drop off wobble goal
        Trajectory trajectoryForwardC = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(48, -60))
                .build();

        drive.setPoseEstimate(startingPosition);
        waitForStart();

        drive.followTrajectoryAsync(trajectoryForwardB); //spot to test to different trajectories

        while (opModeIsActive()) {
            drive.update();
        }
    }
}
