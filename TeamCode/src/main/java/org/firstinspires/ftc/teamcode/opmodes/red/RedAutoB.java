package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RedAutoB")
public class RedAutoB extends LinearOpMode {

    enum State {
        DROP_OFF_WOBBLE_GOAL,
        //SHOOT_POWERSHOTS,
        SHOOT_HIGH_GOAL,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // We define the current state we're on
        // Default to IDLE
        RedAutoB.State currentState = RedAutoB.State.IDLE;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //starting position for robot - halfway across first tile
        Pose2d startingPosition = new Pose2d(-54, -54, Math.toRadians(0)); //maximum starting position
        drive.setPoseEstimate(startingPosition);

        Trajectory dropOffWobbleGoal = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(90,0))
                .build();

        /*Trajectory shootPowershots = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .lineTo(new Vector2d(-33,42))
                .build();*/

        //trajectory for shooting into the high goal
        Trajectory shootHighGoal = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .lineTo(new Vector2d(-33,18))
                .build();

        double turnAngle = Math.toRadians(-90); //turn angle for shooting highshoots

        drive.setPoseEstimate(startingPosition);
        waitForStart();

        //currentState = RedAutoB.State.DROP_OFF_WOBBLE_GOAL;
        drive.followTrajectoryAsync(dropOffWobbleGoal);
        currentState = RedAutoB.State.DROP_OFF_WOBBLE_GOAL;

        while (opModeIsActive()) {
            switch (currentState) {
                case DROP_OFF_WOBBLE_GOAL:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = RedAutoB.State.SHOOT_HIGH_GOAL;
                        drive.followTrajectoryAsync(shootHighGoal);
                        drive.turnAsync(turnAngle);
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
        }
    }
}