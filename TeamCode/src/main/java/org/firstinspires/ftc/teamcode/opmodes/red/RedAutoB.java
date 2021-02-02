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
        PICK_UP_RINGS,
        PICK_UP_SECOND_WOBBLE_ONE,
        PICK_UP_SECOND_WOBBLE_TWO,
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

        //trajectory for turning around and picking up 1 ring
        Trajectory pickUpRings = drive.trajectoryBuilder(shootHighGoal.end())
                .lineTo(new Vector2d(-18,0))
                .build();

        //trajectory for moving to [-48,0] for first part of getting second wobble goal
        Trajectory pickUpSecondWobbleOne = drive.trajectoryBuilder(pickUpRings.end())
                .lineTo(new Vector2d(-33,36))
                .build();

        //trajectory for getting to second wobble goal for pickup - NOTE: may not need to strafe so much b/c we might hit and knock the wobble goal over
        Trajectory pickUpSecondWobbleTwo = drive.trajectoryBuilder(pickUpSecondWobbleOne.end())
                .strafeRight(15)
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
                case SHOOT_HIGH_GOAL:
                    if (!drive.isBusy()) {
                        turnAngle = Math.toRadians(180);
                        drive.turnAsync(turnAngle);
                        currentState = RedAutoB.State.PICK_UP_RINGS;
                        drive.followTrajectoryAsync(pickUpRings);
                    }
                    break;
                case PICK_UP_RINGS:
                    if (!drive.isBusy()) {
                        currentState = RedAutoB.State.PICK_UP_SECOND_WOBBLE_ONE;
                        drive.followTrajectoryAsync(pickUpSecondWobbleOne);
                    }
                    break;
                case PICK_UP_SECOND_WOBBLE_ONE:
                    if (!drive.isBusy()){
                        currentState = RedAutoB.State.PICK_UP_SECOND_WOBBLE_TWO;
                        drive.followTrajectoryAsync(pickUpSecondWobbleTwo);
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