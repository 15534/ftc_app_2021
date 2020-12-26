package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RedAutoC")
public class RedAutoC extends LinearOpMode {

    enum State {
        DROP_OFF_WOBBLE_GOAL,
        //SHOOT_POWERSHOTS,
        SHOOT_HIGH_GOAL,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //constants
        RedAutoC.State currentState = RedAutoC.State.IDLE;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //starting position for robot - halfway across first tile
        Pose2d startingPosition = new Pose2d(-54, -54, Math.toRadians(0)); //maximum starting position
        drive.setPoseEstimate(startingPosition);

        //wobble goal trajectory
        Trajectory dropOffWobbleGoal = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(96,0))
                .build();

        //shoot powershots trajectory
        /*Trajectory shootPowershots = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .lineTo(new Vector2d(39,42))
                .build();*/

        Trajectory shootHighGoal = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .lineTo(new Vector2d(-39,18))
                .build();

        double turnAngle = Math.toRadians(-90); //turn angle for dropping off wobble goal

        currentState = RedAutoC.State.DROP_OFF_WOBBLE_GOAL;
        drive.followTrajectoryAsync(dropOffWobbleGoal); //getting to (42,-54) to drop off wobble goal
        drive.turnAsync(turnAngle); //turning robot to align dropper with tile c

        //loop
        while (opModeIsActive()) {
            switch (currentState) {
                case DROP_OFF_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        currentState = RedAutoC.State.SHOOT_HIGH_GOAL;
                        //drive.followTrajectoryAsync(shootPowershots); //going to launch line to shoot powershots
                        drive.followTrajectoryAsync(shootHighGoal);
                    }
                    break;
            }

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            telemetry.update();
        }
    }
}