package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Wobble;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RedC extends RedAuto {

    double time = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    LinearOpMode op;

    Trajectory launchPosition, dropOffWobbleGoal, pickUp3Rings, goBackToLaunchPosition,
            pickUpRingAndWobbleGoal, pickUpSecondGoal, goBackToLaunchPosition2,
            dropOffSecondWobbleGoal, goOverLaunchLine;

    enum State {
        ACTION_SHOOT_THREE_RINGS,
        GO_TO_WOBBLE_GOAL,
        ACTION_DROP_OFF_WOBBLE_GOAL,
        GO_TO_3_RINGS,
        ACTION_PICK_UP_3_RINGS,
        GO_TO_LAUNCH_POSITION,
        ACTION_SHOOT_THREE_MORE_RINGS,
        PICK_UP_RING_AND_WOBBLE_GOAL,
        PICK_UP_WOBBLE_2,
        ACTION_PICK_UP_WOBBLE_GOAL,
        GO_TO_SHOOTING_POSITION,
        GO_BACK_LAUNCH_LINE,
        ACTION_SHOOT_1_RING,
        DROP_OFF_SECOND_WOBBLE_GOAL,
        ACTION_DROP_OFF_SECOND_WOBBLE_GOAL,
        PARK_OVER_LAUNCH_LINE,
        IDLE
    }

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    public RedC(RedAuto op) {
        this.op = op;
        indexer = op.indexer;
        flap = op.flap;
        transfer = op.transfer;
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

        //Drop off the wobble goal
        dropOffWobbleGoal = drive.trajectoryBuilder(launchPosition.end())
                .splineToSplineHeading(new Pose2d(48, -57, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        //Go back to pick up three more rings from stack
        pickUp3Rings = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(42,-36), Math.toRadians(180))
                //.splineToSplineHeading(new Pose2d(5, -36, Math.toRadians(-180)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-15, -36, Math.toRadians(-180)), Math.toRadians(0))
                .build();

        //getting into a position to drop off second wobble goal
        goBackToLaunchPosition = drive.trajectoryBuilder(pickUp3Rings.end())
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        pickUpRingAndWobbleGoal = drive.trajectoryBuilder(goBackToLaunchPosition.end())
                .splineToSplineHeading(new Pose2d(-20, -36, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-33, -36, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-32, -35), Math.toRadians(0)) //test this again
                .build();

        pickUpSecondGoal = drive.trajectoryBuilderSlow(pickUpRingAndWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(-32, -26), Math.toRadians(-90))
                .build();

        goBackToLaunchPosition2 = drive.trajectoryBuilder(pickUpSecondGoal.end())
                .splineToSplineHeading(new Pose2d(0, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        dropOffSecondWobbleGoal = drive.trajectoryBuilder(goBackToLaunchPosition2.end())
                .splineToSplineHeading(new Pose2d(41, -57, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        goOverLaunchLine = drive.trajectoryBuilder(dropOffSecondWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(41,-39), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(12, -39), Math.toRadians(-90))
                .build();
    }

    public void run() throws InterruptedException {
        runtime.reset();

        next(State.GO_TO_SHOOTING_POSITION);
        drive.followTrajectoryAsync(launchPosition);

        wobble.armDown();
        transfer.setPower(1);

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
                        transfer.setPower(0);
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
                        drive.followTrajectoryAsync(pickUp3Rings);
                        next(State.GO_TO_3_RINGS);
                    }
                    break;
                case GO_TO_3_RINGS:
                    if (!drive.isBusy()) {
                        next(State.ACTION_PICK_UP_3_RINGS);
                    }
                    break;
                case ACTION_PICK_UP_3_RINGS:
                    if (elapsed < 2) {
                        // pick up the rings (not programmed yet)
                    } else {
                        drive.followTrajectoryAsync(goBackToLaunchPosition);
                        next(State.GO_TO_LAUNCH_POSITION);
                    }
                    break;
                case GO_TO_LAUNCH_POSITION:
                    if (!drive.isBusy()) {
                        currentState = State.ACTION_SHOOT_THREE_MORE_RINGS;
                        drive.followTrajectoryAsync(goBackToLaunchPosition);
                        wobble.armDown();
                    }
                    break;
                case ACTION_SHOOT_THREE_MORE_RINGS:
                    if (elapsed < 2) {
                        // shoot the rings (not programmed yet)
                    } else {
                        drive.followTrajectoryAsync(pickUpRingAndWobbleGoal);
                        next(State.PICK_UP_RING_AND_WOBBLE_GOAL);
                    }
                    break;
                case PICK_UP_RING_AND_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(pickUpSecondGoal);
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
                        drive.followTrajectoryAsync(goBackToLaunchPosition2);
                        next(State.GO_BACK_LAUNCH_LINE);
                    }
                    break;
                case GO_BACK_LAUNCH_LINE:
                    if (!drive.isBusy()) {
                        next(State.ACTION_SHOOT_1_RING);
                    }
                    break;
                case ACTION_SHOOT_1_RING:
                    if (elapsed < 2) {
                        // shoot the rings (not programmed yet)
                    } else {
                        drive.followTrajectoryAsync(dropOffSecondWobbleGoal);
                        next(State.DROP_OFF_SECOND_WOBBLE_GOAL);
                    }
                    break;
                case DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (!drive.isBusy()){
                        next(State.ACTION_DROP_OFF_SECOND_WOBBLE_GOAL);
                    }
                    break;
                case ACTION_DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (elapsed < 0.5) {
                        wobble.release();
                    } else {
                        drive.followTrajectoryAsync(goOverLaunchLine);
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