package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RedAuto")
public class RedAuto extends LinearOpMode {

    enum State {
        //trajectories for red a
        TRAJECTORY_A_1,
        TRAJECTORY_A_2,

        //enters idle state when done
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
        Trajectory trajectoryForwardA = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(48, 0))
                .build();

        double turnAngle = Math.toRadians(-90);

        /* code that will get run after dropping off wobble goal in A
        Pose2d positionAfterFirstMovement = trajectoryForwardA.end().plus(new Pose2d (0,0, turnAngle));
        Trajectory trajectoryForwardATwo = drive.trajectoryBuilder(positionAfterFirstMovement)
                .lineToConstantHeading(new Vector2d(0, 0))
                .build();
         */

        /*
        //trajectory to go to tile b to drop off wobble goal
        Trajectory trajectoryForwardB = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(24, -36)) //maybe need to change lineTo for this
                .build();

        //trajectory to go to tile b to drop off wobble goal
        Trajectory trajectoryForwardC = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(48, -60))
                .build();
        */

        drive.setPoseEstimate(startingPosition);
        waitForStart();

        currentState = State.TRAJECTORY_A_1; //need to convert this to if based off of computer vision

        while (opModeIsActive()) {
            // this will also need to be modified after computer vision
            switch (currentState) {
                // Check if the drive class isn't busy
                // `isBusy() == true` while it's following the trajectory
                // Once `isBusy() == false`, the trajectory follower signals that it is finished
                // We move on to the next state
                // Make sure we use the async follow function
                case TRAJECTORY_A_1:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_A_2;
                        drive.turnAsync(turnAngle);
                    }
                    break;

                //code for movement after dropping off wobble goal in A
                /*case TRAJECTORY_A_2:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_A_2;
                    }
                    break;
                 */
            }
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
