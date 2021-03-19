package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RedAutoCNew")
public class RedAutoCNew extends LinearOpMode {

    enum State {
        SHOOT_THREE_RINGS,
        ACTION_SHOOT_THREE_RINGS,
        DROP_OFF_WOBBLE_GOAL,
        ACTION_DROP_OFF_WOBBLE_GOAL,
        PICK_UP_3_RINGS,
        ACTION_PICK_UP_3_RINGS,
        SHOOT_THREE_MORE_RINGS,
        ACTION_SHOOT_THREE_MORE_RINGS,
        PICK_UP_ONE_RING_AND_WOBBLE_GOAL,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //constants
        State currentState = State.IDLE;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //starting position for robot - halfway across first tile
        Pose2d startingPosition = new Pose2d(-63, -57, Math.toRadians(0)); //maximum starting position
        drive.setPoseEstimate(startingPosition);

        //Go forward to intermediate point.
        Trajectory launchPosition = drive.trajectoryBuilder(startingPosition)
                .splineToConstantHeading(new Vector2d(-18, -57), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, -36), Math.toRadians(0))
                .build();

        //Drop off the wobble goal.
        Trajectory dropOffWobbleGoal = drive.trajectoryBuilder(launchPosition.end())
                .splineToSplineHeading(new Pose2d(48, -48, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        //Go back to pick up three more rings from stack.
        Trajectory pickUp3Rings = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .splineToSplineHeading(new Pose2d(5, -36, Math.toRadians(-180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-15, -36), Math.toRadians(0))
                .build();

        //getting into a position to drop off second wobble goal
        Trajectory goBackToLaunchPosition = drive.trajectoryBuilder(pickUp3Rings.end())
                .splineToSplineHeading(new Pose2d(0, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        //getting into a position to drop off second wobble goal
        Trajectory pickUpRingAndWobbleGoal = drive.trajectoryBuilder(goBackToLaunchPosition.end())
                .splineToSplineHeading(new Pose2d(-20, -36, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-31, -28, Math.toRadians(90)), Math.toRadians(0))
                .build();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        double time = 0.0;

        currentState = State.SHOOT_THREE_RINGS;
        drive.followTrajectoryAsync(launchPosition);

        //loop
        while (opModeIsActive()) {
            switch (currentState) {
                case SHOOT_THREE_RINGS:
                    if (!drive.isBusy()) {
                        currentState = State.ACTION_SHOOT_THREE_RINGS;
                        drive.followTrajectoryAsync(dropOffWobbleGoal);
                        time = runtime.seconds();
                    }
                    break;
                case ACTION_SHOOT_THREE_RINGS:
                    if (runtime.seconds() - time > 3) {
                        currentState = State.DROP_OFF_WOBBLE_GOAL;
                    }
                    break;
                case DROP_OFF_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        currentState = State.ACTION_DROP_OFF_WOBBLE_GOAL;
                        drive.followTrajectoryAsync(pickUp3Rings);
                        time = runtime.seconds();
                    }
                    break;
                case ACTION_DROP_OFF_WOBBLE_GOAL:
                    if (runtime.seconds() - time > 3) {
                        currentState = State.PICK_UP_3_RINGS;
                    }
                    break;
                case PICK_UP_3_RINGS:
                    if (!drive.isBusy()) {
                        currentState = State.ACTION_PICK_UP_3_RINGS;
                        drive.followTrajectoryAsync(goBackToLaunchPosition);
                        time = runtime.seconds();
                    }
                    break;
                case ACTION_PICK_UP_3_RINGS:
                    if (runtime.seconds() - time > 3) {
                        currentState = State.SHOOT_THREE_MORE_RINGS;
                    }
                    break;
                case SHOOT_THREE_MORE_RINGS:
                    if (!drive.isBusy()) {
                        currentState = State.ACTION_SHOOT_THREE_MORE_RINGS;
                        drive.followTrajectoryAsync(pickUpRingAndWobbleGoal);
                        time = runtime.seconds();
                    }
                    break;
                case ACTION_SHOOT_THREE_MORE_RINGS:
                    if (runtime.seconds() - time > 3) {
                        currentState = State.PICK_UP_ONE_RING_AND_WOBBLE_GOAL;
                    }
                    break;
                case PICK_UP_ONE_RING_AND_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
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
