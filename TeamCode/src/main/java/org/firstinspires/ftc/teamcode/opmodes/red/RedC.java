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

@Autonomous(name="RedC")
public class RedC extends RedAuto {

    double time = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    LinearOpMode op;

    Trajectory launchPosition, dropOffWobbleGoal, pickUp3RingsIntermediatePoint, pickUp3Rings, goBackToLaunchPosition,
            pickUpRingAndWobbleGoal, getInPositionForSecondWobbleGoal, pickUpSecondGoal, goToShooter, shootHighGoal, goBackToLaunchPosition2,
            dropOffSecondWobbleGoal, goOverLaunchLine;

    enum State {
        ACTION_SHOOT_THREE_RINGS,
        GO_TO_WOBBLE_GOAL,
        ACTION_DROP_OFF_WOBBLE_GOAL,
        INTERMEIDATE_POINT,
        GO_TO_3_RINGS,
        PICK_UP_THREE_RINGS,
        ACTION_PICK_UP_3_RINGS,
        GO_TO_LAUNCH_POSITION,
        ACTION_SHOOT_THREE_MORE_RINGS,
        PICK_UP_RING_AND_WOBBLE_GOAL,
        ALIGN_WOBBLE_GOAL,
        PICK_UP_WOBBLE_2,
        ACTION_PICK_UP_WOBBLE_GOAL,
        GO_TO_SHOOTING_POSITION,
        GO_BACK_LAUNCH_LINE,
        ACTION_SHOOT_1_RING,
        DROP_OFF_SECOND_WOBBLE_GOAL,
        ACTION_DROP_OFF_SECOND_WOBBLE_GOAL,
        GO_TO_SHOOTING_POSITION_2,
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
        intake = op.intake;
        flap = op.flap;
        drive = op.drive;
        shooter = op.shooter;
        wobble = op.wobble;
        telemetry = op.telemetry;
    }

    public void buildTrajectories() {
        //Go forward to intermediate point
        //startingpos = (-63,-57)
        launchPosition = drive.trajectoryBuilder(startingPosition)
                //.addTemporalMarker(0.4, shooter::activate)
                .addTemporalMarker(0.5, () -> {
                    if (useShooter) {
                        shooter.activate();
                    }
                })
                .addTemporalMarker(1, () -> {
                    if (useShooter) {
                        indexer.setPower(1);
                        intake.setPower(1);
                    }
                })
                //.addTemporalMarker(1.6, shooter::allow)
                //.addTemporalMarker(1, shooter::push)
                .splineToConstantHeading(new Vector2d(-15, -57), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(0))
                .build();

        //Drop off the wobble goal
        dropOffWobbleGoal = drive.trajectoryBuilder(launchPosition.end())
                .splineToSplineHeading(new Pose2d(48, -56, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        //Go back to pick up three more rings from stack
        pickUp3RingsIntermediatePoint = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .addTemporalMarker(1, wobble::armMiddle)
                .splineToConstantHeading(new Vector2d(48,-34), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-6, -48.75, Math.toRadians(125)), Math.toRadians(0))
                //.splineToSplineHeading(new Pose2d(-10.71, -54.34, Math.toRadians(120)), Math.toRadians(0))
                .build();

        pickUp3Rings = drive.trajectoryBuilderSlow(pickUp3RingsIntermediatePoint.end())
//                .lineTo(new Vector2d(-31.1,-10.3))
//                .addSpatialMarker(new Vector2d(-31.1, -10.3), () -> {
//                    intake.setPower(1);
//                    indexer.setPower(1);
//                })
                .lineTo(new Vector2d(-32.66,-16.67))
                .addSpatialMarker(new Vector2d(-32.66, -16.67), () -> {
                    intake.setPower(1);
                    indexer.setPower(1);
                })
//                .splineToSplineHeading(new Pose2d(-34, -36.5, Math.toRadians(90)), Math.toRadians(90))
                .build();

        //getting into a position to drop off second wobble goal
//        goBackToLaunchPosition = drive.trajectoryBuilder(pickUp3Rings.end())
//                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(0)), Math.toRadians(0))
//                .build();

//        pickUpRingAndWobbleGoal = drive.trajectoryBuilder(goBackToLaunchPosition.end())
//                .splineToSplineHeading(new Pose2d(-20, -36, Math.toRadians(180)), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(-33, -36, Math.toRadians(90)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-32, -35), Math.toRadians(0)) //test this again
//                .build();

        getInPositionForSecondWobbleGoal = drive.trajectoryBuilder(pickUp3Rings.end(), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-25.1, -10.3), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-25.1,-36, Math.toRadians(90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-33.5,-36), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(-33.5,-36), () -> {
                    wobble.armDown();
                    intake.setPower(0);
                    indexer.setPower(0);
                })
                .build();

        pickUpSecondGoal = drive.trajectoryBuilder(getInPositionForSecondWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(-33.5, -24.25), Math.toRadians(90))
                .build();

//        goBackToLaunchPosition2 = drive.trajectoryBuilder(pickUpSecondGoal.end())
//                .splineToSplineHeading(new Pose2d(0, -36, Math.toRadians(0)), Math.toRadians(0))
//                .build();

        goToShooter = drive.trajectoryBuilder(pickUpSecondGoal.end())
                .splineToSplineHeading(new Pose2d(0,-36,Math.toRadians(0)), Math.toRadians(0))
                .build();

        dropOffSecondWobbleGoal = drive.trajectoryBuilder(pickUpSecondGoal.end(), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-6.5, -56, Math.toRadians(15)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(42, -57, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        goOverLaunchLine = drive.trajectoryBuilder(dropOffSecondWobbleGoal.end())
                .addTemporalMarker(1, wobble::armMiddle)
                .splineToConstantHeading(new Vector2d(42, -39), Math.toRadians(-90))
                //.splineToConstantHeading(new Vector2d(41,-39), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(12, -39), Math.toRadians(-90))
                .build();
    }

    public void run() throws InterruptedException {
        runtime.reset();

        next(State.GO_TO_SHOOTING_POSITION);
        drive.followTrajectoryAsync(launchPosition);

        //loop
        while (op.opModeIsActive()) {
            double elapsed = runtime.seconds() - time;
            switch (currentState) {
                case GO_TO_SHOOTING_POSITION:
                    if (!drive.isBusy()) {
                        next(State.ACTION_SHOOT_THREE_RINGS);
                        flap.setPosition(0.205);
                    }
                    break;
                case ACTION_SHOOT_THREE_RINGS:
                    double shootTime = 0;
                    if (useShooter) {
                        if (elapsed < shootTime + 0.1) {
                            shooter.allow();
//                            intake.setPower(1);
//                            indexer.setPower(1);
                        } else if (elapsed < shootTime + 0.3) {
                            shooter.push();
                        } else if (elapsed < shootTime + 0.7) {
                            shooter.release();
                        } else if (elapsed < shootTime + 2.3) {
                            shooter.push();
                        } else if (elapsed < shootTime + 2.7) {
                            shooter.release();
                        }
//                        else if (elapsed < shootTime + 0.8) {
//                            shooter.push();
//                        } else if (elapsed < shootTime + 1.0) {
//                            shooter.release();
//                        } else if (elapsed < shootTime + 1.3) {
//                            shooter.push();
//                        } else if (elapsed < shootTime + 1.5) {
//                            shooter.release();
//                        } else if (elapsed < shootTime + 1.8) {
//                            shooter.push();
//                        } else if (elapsed < shootTime + 2) {
//                            shooter.release();
//                        }
                        else if (elapsed < shootTime + 6) {
                            //shooter.release();
                            shooter.deactivate();
                            intake.setPower(0);
                            indexer.setPower(0);
//                            wobble.armDown();
//                            wobble.loosen();
                            next(State.IDLE);
//                            next(State.GO_TO_WOBBLE_GOAL);
//                            drive.followTrajectoryAsync(dropOffWobbleGoal);
                        }
                    } else {
                        if (elapsed > 1) {
                            next(State.GO_TO_WOBBLE_GOAL);
                            wobble.armDown();
                            wobble.loosen();
                            drive.followTrajectoryAsync(dropOffWobbleGoal);
                        }
                    }
                    break;
                case GO_TO_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        next(State.ACTION_DROP_OFF_WOBBLE_GOAL);
                    }
                    break;
                case ACTION_DROP_OFF_WOBBLE_GOAL:
                    if (elapsed < 0.3) {
                        wobble.release();
                    } else {
                        drive.followTrajectoryAsync(pickUp3RingsIntermediatePoint);
                        next(State.GO_TO_3_RINGS);
                    }
                    break;
                case GO_TO_3_RINGS:
                    if (!drive.isBusy()) {
                        intake.setPower(1);
                        indexer.setPower(1);
                        drive.followTrajectoryAsync(pickUp3Rings);
                        shooter.block();
                        next(State.PICK_UP_THREE_RINGS);
                    }
                    break;
                case PICK_UP_THREE_RINGS:
                    if (!drive.isBusy()) {
                        intake.setPower(0);
                        indexer.setPower(0);
                        drive.followTrajectoryAsync(getInPositionForSecondWobbleGoal);
                        next(State.ALIGN_WOBBLE_GOAL);
                    }
                    break;
                case ALIGN_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(pickUpSecondGoal);
                        next(State.PICK_UP_WOBBLE_2);
                    }
                    break;
                // TODO the rest isn't really programmed, but we should pick up the wobble goal
                //  from here instead of shooting
//
//                case ACTION_PICK_UP_3_RINGS:
//                    if (elapsed < 2) {
//                        // pick up the rings (not programmed yet)
//                    } else {
//                        drive.followTrajectoryAsync(goBackToLaunchPosition);
//                        next(State.GO_TO_LAUNCH_POSITION);
//                    }
//                    break;
//                case GO_TO_LAUNCH_POSITION:
//                    if (!drive.isBusy()) {
//                        currentState = State.ACTION_SHOOT_THREE_MORE_RINGS;
//                        drive.followTrajectoryAsync(goBackToLaunchPosition);
//                    }
//                    break;
//                case ACTION_SHOOT_THREE_MORE_RINGS:
//                    if (elapsed < 2) {
//                        // shoot the rings (not programmed yet)
//                    } else {
//                        wobble.armDown();
//                        drive.followTrajectoryAsync(pickUpRingAndWobbleGoal);
//                        next(State.PICK_UP_RING_AND_WOBBLE_GOAL);
//                    }
//                    break;
//                case PICK_UP_RING_AND_WOBBLE_GOAL:
//                    if (!drive.isBusy()) {
//                        drive.followTrajectoryAsync(pickUpSecondGoal);
//                        next(State.PICK_UP_WOBBLE_2);
//                    }
//                    break;
                case PICK_UP_WOBBLE_2:
                    if (!drive.isBusy()) {
                        next(State.ACTION_PICK_UP_WOBBLE_GOAL);
                    }
                    break;
                case ACTION_PICK_UP_WOBBLE_GOAL:
                    if (elapsed < 0.3) {
                        wobble.grip();
                    } else {
                        drive.followTrajectoryAsync(dropOffSecondWobbleGoal);
                        next(State.DROP_OFF_SECOND_WOBBLE_GOAL);
                    }
                    break;
//                case GO_BACK_LAUNCH_LINE:
//                    if (!drive.isBusy()) {
//                        next(State.ACTION_SHOOT_1_RING);
//                    }
//                    break;
//                case ACTION_SHOOT_1_RING:
//                    if (elapsed < 2) {
//                        // shoot the rings (not programmed yet)
//                    } else {
//                        drive.followTrajectoryAsync(dropOffSecondWobbleGoal);
//                        next(State.DROP_OFF_SECOND_WOBBLE_GOAL);
//                    }
//                    break;
//                case GO_TO_SHOOTING_POSITION_2:
//                    if(!drive.isBusy()) {
//                        drive.followTrajectoryAsync(goToShooter);
//                        next(State.DROP_OFF_SECOND_WOBBLE_GOAL);
//                    }
//                    break;
                case DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        next(State.ACTION_DROP_OFF_SECOND_WOBBLE_GOAL);
                        wobble.loosen();
                    }
                    break;
                case ACTION_DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (elapsed < 0.3) {
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