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
public class RedAutoCNew extends LinearOpMode {

    enum State {
        ACTION_SHOOT_THREE_RINGS,
        GO_TO_WOBBLE_GOAL_TRANSITION,
        GO_TO_WOBBLE_GOAL,
        ACTION_DROP_OFF_WOBBLE_GOAL,
        GO_TO_3_RINGS_TRANSITION,
        GO_TO_3_RINGS,
        ACTION_PICK_UP_3_RINGS,
        GO_TO_LAUNCH_POSITION,
        ACTION_SHOOT_THREE_MORE_RINGS,
        PICK_UP_RING_AND_WOBBLE_GOAL,
        ACTION_PICK_UP_WOBBLE_GOAL,
        GO_BACK_LAUNCH_LINE,
        ACTION_SHOOT_1_RING,
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
                .splineToSplineHeading(new Pose2d(48, -48, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        //Go back to pick up three more rings from stack
        Trajectory pickUp3Rings = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(48,-43), Math.toRadians(0))
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
                .splineToSplineHeading(new Pose2d(-33, -36, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-29.5, -27), Math.toRadians(0)) //test this again
                .build();

        Trajectory goBackToLaunchPosition2 = drive.trajectoryBuilder(pickUpRingAndWobbleGoal.end())
                .splineToSplineHeading(new Pose2d(0, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory dropOffSecondWobbleGoal = drive.trajectoryBuilder(goBackToLaunchPosition2.end())
                .splineToSplineHeading(new Pose2d(48, -48, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        Trajectory goOverLaunchLine = drive.trajectoryBuilder(dropOffSecondWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(12,-48), Math.toRadians(0))
                .build();

        //wobble.armUp();
//        wobble.grip();
//        wobble.armDown();

        wobble.armUp();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        double time = 0.0;

        currentState = State.ACTION_SHOOT_THREE_RINGS;
        drive.followTrajectoryAsync(launchPosition);

        wobble.grip();
        wobble.armDown();

        //loop
        while (opModeIsActive()) {
            switch (currentState) {
                case ACTION_SHOOT_THREE_RINGS:
                    if (runtime.seconds() - time > 3) {
                        //shoot the rings
                        currentState = State.GO_TO_WOBBLE_GOAL;
                    }
                    break;
                case GO_TO_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        currentState = State.ACTION_DROP_OFF_WOBBLE_GOAL;
                        drive.followTrajectoryAsync(dropOffWobbleGoal);
                    }
                    break;
                case ACTION_DROP_OFF_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        //drop off the wobble goal
                        wobble.release();
                        currentState = State.GO_TO_3_RINGS;
                    }
                    break;
                case GO_TO_3_RINGS:
                    if (!drive.isBusy()) {
                        currentState = State.ACTION_PICK_UP_3_RINGS;
                        drive.followTrajectoryAsync(pickUp3Rings);
                    }
                    break;
                case ACTION_PICK_UP_3_RINGS:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        // need to add code to pick up the rings
                    }
                    break;
                case GO_TO_LAUNCH_POSITION:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(goBackToLaunchPosition);
                        currentState = State.ACTION_SHOOT_THREE_MORE_RINGS;
                        time = runtime.seconds();
                        wobble.armDown();
                    }
                    break;
                case ACTION_SHOOT_THREE_MORE_RINGS:
                    if (runtime.seconds() - time > 3) {
                        //nned to add code to shoot the rings
                        currentState = State.PICK_UP_RING_AND_WOBBLE_GOAL;
                    }
                    break;
                case PICK_UP_RING_AND_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        wobble.release();
                        drive.followTrajectoryAsync(pickUpRingAndWobbleGoal);
                        time = runtime.seconds();
                        currentState = State.ACTION_PICK_UP_WOBBLE_GOAL;
                    }
                    break;
                case ACTION_PICK_UP_WOBBLE_GOAL:
                    if (runtime.seconds() - time < 0.5) {
                        wobble.grip();
                        wobble.armUp();
                        currentState = State.IDLE; //change back later
                    }
                    break;
                case GO_BACK_LAUNCH_LINE:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(goBackToLaunchPosition2);
                        time = runtime.seconds();
                        currentState = State.IDLE; //change back later
                    }
                    break;
                case ACTION_SHOOT_1_RING:
                    if (runtime.seconds() - time > 3) {
                        currentState = State.DROP_OFF_SECOND_WOBBLE_GOAL;
                    }
                case DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (!drive.isBusy()){
                        currentState = State.ACTION_DROP_OFF_SECOND_WOBBLE_GOAL;
                        drive.followTrajectoryAsync(dropOffSecondWobbleGoal);
                        time = runtime.seconds();
                    }
                    break;
                case ACTION_DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (runtime.seconds() - time > 3) {
                        currentState = State.PARK_OVER_LAUNCH_LINE;
                        //wobble.armDown();
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
