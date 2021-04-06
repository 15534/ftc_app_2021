package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;

public class RedA extends RedAuto {

    double time = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    LinearOpMode op;

    Trajectory launchPosition, dropOffWobbleGoal, intermediatePoint, pickUpSecondWobbleGoal,
            dropOffSecondWobbleGoal, parkOverLaunchLine;


    enum State {
        GO_TO_SHOOTING_POSITION,
        ACTION_SHOOT_THREE_RINGS,
        GO_TO_WOBBLE_GOAL,
        ACTION_DROP_OFF_WOBBLE_GOAL,
        INTERMEDIATE_POINT,
        PICK_UP_WOBBLE_2,
        ACTION_PICK_UP_WOBBLE_GOAL,
        DROP_OFF_SECOND_WOBBLE_GOAL,
        PARK_OVER_LAUNCH_LINE,
        IDLE
    }

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    public RedA(RedAuto op) {
        this.op = op;
        indexer = op.indexer;
        flap = op.flap;
        drive = op.drive;
        shooter = op.shooter;
        wobble = op.wobble;
        telemetry = op.telemetry;
    }

    public void buildTrajectories() {
        //Go forward to intermediate point
        launchPosition = drive.trajectoryBuilder(startingPosition)
                .addTemporalMarker(0.6, () -> {
                    shooter.activate();
                    indexer.setPower(1);
                })
                .addTemporalMarker(1, shooter::push)
                .splineToConstantHeading(new Vector2d(-18, -57), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, -36), Math.toRadians(0))
                .build();

        dropOffWobbleGoal = drive.trajectoryBuilder(launchPosition.end())
                .splineToSplineHeading(new Pose2d(12,-57, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        intermediatePoint = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(12, -55), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-48, -48, Math.toRadians(90)), Math.toRadians(0))
                .build();

        pickUpSecondWobbleGoal = drive.trajectoryBuilderSlow(intermediatePoint.end())
                .splineToConstantHeading(new Vector2d(-48, -36), Math.toRadians(0))
                .build();

        dropOffSecondWobbleGoal = drive.trajectoryBuilder(pickUpSecondWobbleGoal.end())
                .splineToSplineHeading(new Pose2d(6,-48, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        parkOverLaunchLine = drive.trajectoryBuilder(dropOffSecondWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(6,-42), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(12,-42), Math.toRadians(0))
                .build();

    }

    public void run() throws InterruptedException {
        runtime.reset();

        next(State.GO_TO_SHOOTING_POSITION);
        drive.followTrajectoryAsync(launchPosition);

        wobble.armDown();

        //loop
        while (op.opModeIsActive()) {
            double elapsed = runtime.seconds() - time;
            switch (currentState) {
                case GO_TO_SHOOTING_POSITION:
                    if (!drive.isBusy()) {
                        next(State.ACTION_SHOOT_THREE_RINGS);
                    }
                    break;
                case ACTION_SHOOT_THREE_RINGS:
                    if (elapsed < 0.2) {
                        shooter.push();
                    } else if (elapsed < 0.4) {
                        shooter.release();
                    } else if (elapsed < 0.6) {
                        shooter.push();
                    } else if (elapsed < 0.8) {
                        shooter.release();
                    } else if (elapsed < 1) {
                        shooter.push();
                    } else if (elapsed < 1.2) {
                        shooter.release();
                    } else {
                        shooter.deactivate();
                        shooter.block();
                        indexer.setPower(0);
                        next(State.GO_TO_WOBBLE_GOAL);
                        drive.followTrajectoryAsync(dropOffWobbleGoal);
                    }
                    break;
                case GO_TO_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        next(State.ACTION_DROP_OFF_WOBBLE_GOAL);
                    }
                    break;
                case ACTION_DROP_OFF_WOBBLE_GOAL:
                    if (elapsed < 0.5) {
                        wobble.release();
                    } else {
                        drive.followTrajectoryAsync(intermediatePoint);
                        next(State.INTERMEDIATE_POINT);
                    }
                    break;
                case INTERMEDIATE_POINT:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(pickUpSecondWobbleGoal);
                        next(State.PICK_UP_WOBBLE_2);
                    }
                    break;
                case PICK_UP_WOBBLE_2:
                    if (!drive.isBusy()) {
                        next(State.ACTION_PICK_UP_WOBBLE_GOAL);
                    }
                    break;
                case ACTION_PICK_UP_WOBBLE_GOAL:
                    if (elapsed < 0.5) {
                        wobble.grip();
                    } else {
                        drive.followTrajectoryAsync(dropOffSecondWobbleGoal);
                        next(State.DROP_OFF_SECOND_WOBBLE_GOAL);
                    }
                    break;
                case DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (elapsed < 0.5) {
                        wobble.release();
                    } else {
                        drive.followTrajectoryAsync(parkOverLaunchLine);
                        next(State.PARK_OVER_LAUNCH_LINE);
                    }
            }

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("state", currentState);
            telemetry.addData("elapsed", elapsed);
            telemetry.update();

        }
    }
}
