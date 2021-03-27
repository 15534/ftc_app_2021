package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Wobble;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RedAutoCNew")
public class RedAutoBNew extends LinearOpMode {

    enum State {
        ACTION_SHOOT_ONE_RING,
        GO_TO_WOBBLE_GOAL,
        ACTION_DROP_OFF_WOBBLE_GOAL,
        GO_TO_ONE_RING,
        ACTION_PICK_UP_ONE_RING,
        GO_TO_LAUNCH_POSITION,
        ACTION_SHOOT_ONE_MORE_RING,
        PICK_UP_RING_AND_WOBBLE_GOAL,
        ACTION_PICK_UP_WOBBLE_GOAL,
        DROP_OFF_SECOND_WOBBLE_GOAL,
        ACTION_DROP_OFF_SECOND_WOBBLE_GOAL,
        PARK_OVER_LAUNCH_LINE,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //constants
        State currentState = State.IDLE;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Wobble wobble = new Wobble(hardwareMap);

        //starting position for robot - halfway across first tile
        Pose2d startingPosition = new Pose2d(-63, -57, Math.toRadians(0)); //maximum starting position
        drive.setPoseEstimate(startingPosition);

        //Go forward to intermediate point
        Trajectory launchPosition = drive.trajectoryBuilder(startingPosition)
                .splineToConstantHeading(new Vector2d(-18, -57), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, -36), Math.toRadians(0))
                .build();

        //Drop off the wobble goal
        Trajectory dropOffWobbleGoal = drive.trajectoryBuilder(launchPosition.end())
                .splineToConstantHeading(new Vector2d(-36, -57), Math.toRadians(0))
                .build();

        Trajectory pickUp1Ring = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .splineToSplineHeading(new Pose2d(5, -36, Math.toRadians(-180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-15, -36), Math.toRadians(0))
                .build();

        //getting into a position to drop off second wobble goal
        Trajectory goBackToLaunchPosition = drive.trajectoryBuilder(pickUp1Ring.end())
                .splineToSplineHeading(new Pose2d(0, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        //getting into a position to drop off second wobble goal
        Trajectory pickUpRingAndWobbleGoal = drive.trajectoryBuilder(goBackToLaunchPosition.end())
                .splineToSplineHeading(new Pose2d(-20, -36, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-33, -36, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-33, -24), Math.toRadians(0))
                .build();

        Trajectory goBackToLaunchPosition2 = drive.trajectoryBuilder(pickUpRingAndWobbleGoal.end())
                .splineToSplineHeading(new Pose2d(0, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory dropOffSecondWobbleGoal = drive.trajectoryBuilder(goBackToLaunchPosition2.end())
                .splineToSplineHeading(new Pose2d(36, -57, Math.toRadians(90)), Math.toRadians(0))
                .build();

        Trajectory goOverLaunchLine = drive.trajectoryBuilder(dropOffSecondWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(12,-48), Math.toRadians(0))
                .build();

        wobble.armUp();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        double time = 0.0;

        drive.followTrajectoryAsync(launchPosition);
        currentState = State.ACTION_SHOOT_ONE_RING;

        //loop
        while (opModeIsActive()) {
            switch (currentState) {
                case ACTION_SHOOT_ONE_RING:
                    if (!drive.isBusy()) {
                        //shoot the rings
                        currentState = State.GO_TO_WOBBLE_GOAL;
                    }
                    break;
                case GO_TO_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(dropOffWobbleGoal);
                        currentState = State.ACTION_DROP_OFF_WOBBLE_GOAL;
                        time = runtime.seconds();
                    }
                    break;
                case ACTION_DROP_OFF_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        //drop off the wobble goal
                        currentState = State.GO_TO_ONE_RING;
                        wobble.armDown();
                        wobble.release();
                        wobble.armUp();
                    }
                    break;
                case GO_TO_ONE_RING:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(pickUp1Ring);
                        currentState = State.ACTION_PICK_UP_ONE_RING;
                        time = runtime.seconds();
                    }
                    break;
                case ACTION_PICK_UP_ONE_RING:
                    if (!drive.isBusy()) {
                        currentState = State.GO_TO_LAUNCH_POSITION;
                    }
                    break;
                case GO_TO_LAUNCH_POSITION:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(goBackToLaunchPosition);
                        currentState = State.ACTION_SHOOT_ONE_MORE_RING;
                        time = runtime.seconds();
                        wobble.armDown();
                    }
                    break;
                case ACTION_SHOOT_ONE_MORE_RING:
                    if (!drive.isBusy()) {
                        //shoot the rings
                        currentState = State.PICK_UP_RING_AND_WOBBLE_GOAL;
                    }
                    break;
                case PICK_UP_RING_AND_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(pickUpRingAndWobbleGoal);
                        time = runtime.seconds();
                        currentState = State.ACTION_PICK_UP_WOBBLE_GOAL;
                    }
                    break;
                case ACTION_PICK_UP_WOBBLE_GOAL:
                    if (!drive.isBusy()){
                        wobble.grip();
                        wobble.armUp();
                        currentState = State.DROP_OFF_SECOND_WOBBLE_GOAL; //change back later
                    }
                    break;
                case DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (!drive.isBusy()){
                        currentState = State.ACTION_DROP_OFF_SECOND_WOBBLE_GOAL;
                        drive.followTrajectoryAsync(goOverLaunchLine);
                        time = runtime.seconds();
                    }
                    break;
                case ACTION_DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        currentState = State.PARK_OVER_LAUNCH_LINE;
                        wobble.armDown();
                        wobble.release();
                        wobble.armUp();
                    }
                    break;
                case PARK_OVER_LAUNCH_LINE:
                    if (!drive.isBusy()){
                        currentState = State.IDLE;
                        drive.followTrajectoryAsync(goOverLaunchLine);
                        time = runtime.seconds();
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
