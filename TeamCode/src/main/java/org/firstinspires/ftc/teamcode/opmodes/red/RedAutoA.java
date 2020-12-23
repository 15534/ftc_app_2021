package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RedAutoA")
public class RedAutoA extends LinearOpMode {

    enum State {
        DROP_OFF_WOBBLE_GOAL,
        //SHOOT_POWERSHOTS,
        SHOOT_HIGH_GOAL,
        DROP_OFF_BLUE_WOBBLE_GOAL,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // We define the current state we're on
        // Default to IDLE
        State currentState = State.IDLE;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //starting position for robot - halfway across first tile
        Pose2d startingPosition = new Pose2d(-54, -54, Math.toRadians(0)); //maximum starting position
        drive.setPoseEstimate(startingPosition);

        //trajectory to go to tile a to drop off wobble goal
        //Pose2d positionForA = new Pose2d(-54, -54, Math.toRadians(-90)); //new position if we have to run trajectory a, robot rotates -90 so we can strafe
        Trajectory dropOffWobbleGoal = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(48, 0))
                .build();

        double turnAngle = Math.toRadians(-90);

        //getting to position to shoot powershots
        /*Trajectory shootPowershots = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                //rerotating as robot moves to spot to shoot powershots,
                //x = -12 to be safe so that bot doesn't go over launch line
                .lineTo(new Vector2d(11,42)) //we could also turn and then strafe (strafeLeft or strafeRight)
                .build();*/

        Trajectory shootHighGoal = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .lineTo(new Vector2d(11,30))
                .build();

        drive.setPoseEstimate(startingPosition);
        waitForStart();

        currentState = State.DROP_OFF_WOBBLE_GOAL;
        drive.followTrajectoryAsync(dropOffWobbleGoal);
        drive.turnAsync(turnAngle);

        while (opModeIsActive()) {
            switch (currentState) {
                case DROP_OFF_WOBBLE_GOAL:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        //currentState = State.SHOOT_POWERSHOTS;
                        //drive.followTrajectoryAsync(shootPowershots);
                        currentState = State.SHOOT_HIGH_GOAL;
                        turnAngle = 90;
                        drive.turnAsync(turnAngle);
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
        }

    }
}

