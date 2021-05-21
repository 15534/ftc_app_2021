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

//@Autonomous(name="RedC")
public class RedA extends RedAuto {

    double time = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    LinearOpMode op;

    Trajectory launchPosition, dropOffWobbleGoal, pickUp3RingsIntermediatePoint, pickUp3Rings, goBackToLaunchPosition,
            pickUpRingAndWobbleGoal, getInPositionForSecondWobbleGoal, pickUpSecondGoal, goToShooter, shootHighGoal, shoot2MoreRings,
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
        ACTION_SHOOT_2_RINGS,
        IDLE
    }

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    public RedA(RedAuto op) {
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
                .splineToSplineHeading(new Pose2d(0, -57, Math.toRadians(13)), Math.toRadians(0))
                .build();

        //Drop off the wobble goal
        dropOffWobbleGoal = drive.trajectoryBuilder(launchPosition.end().plus(new Pose2d(0,0, Math.toRadians(20))))
                .splineToSplineHeading(new Pose2d(2, -54, Math.toRadians(-90)), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(1, -54), wobble::release)
                .build();

        double intakeAngle = Math.toRadians(110);
        double intakeDist = 45;
        double intakeX = intakeDist * Math.cos(intakeAngle);
        double intakeY = intakeDist * Math.sin(intakeAngle);

        //Go back to pick up three more rings from stack
        pickUp3RingsIntermediatePoint = drive.trajectoryBuilder(dropOffWobbleGoal.end(), Math.toRadians(90))
                .addTemporalMarker(1, wobble::armMiddle)
                .splineToConstantHeading(new Vector2d(2,-34), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-13.5159, -49.479, Math.toRadians(110)), Math.toRadians(40))
                .build();

        pickUp3Rings = drive.trajectoryBuilder(pickUp3RingsIntermediatePoint.end())
                .addTemporalMarker(0, () -> {
                    intake.setPower(1);
                    indexer.setPower(1);
                })
                .lineTo(pickUp3RingsIntermediatePoint.end().vec().plus(new Vector2d(intakeX, intakeY)))
                .build();

//        getInPositionForSecondWobbleGoal = drive.trajectoryBuilderSlow(pickUp3Rings.end(), Math.toRadians(-70))
//                .splineToConstantHeading(new Vector2d(-31.75, -21.4), Math.toRadians(110))
//                .build();

        getInPositionForSecondWobbleGoal = drive.trajectoryBuilder(pickUp3Rings.end(), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-25.1, -10.3), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(pickUp3Rings.end().getX(),-36, Math.toRadians(87)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-33.5,-36), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(-33.5,-36), () -> {
                    wobble.armDown();
                    intake.setPower(0);
                })
                .build();

        pickUpSecondGoal = drive.trajectoryBuilder(getInPositionForSecondWobbleGoal.end())
                .splineToConstantHeading(new Vector2d(-33.5, -20), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(-33.5, -20), () -> {
                    indexer.setPower(0);
                })
                .build();

        shoot2MoreRings = drive.trajectoryBuilder(pickUpSecondGoal.end(), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(-5, -40, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(-5, -20), Math.toRadians(-10))
                .build();

//        dropOffSecondWobbleGoal = drive.trajectoryBuilder(shoot2MoreRings.end())
//                .splineToSplineHeading(new Pose2d(42, -57, Math.toRadians(-90)), Math.toRadians(0))
//                .build();

        dropOffSecondWobbleGoal = drive.trajectoryBuilder(shoot2MoreRings.end(), Math.toRadians(-10))
                .splineTo(new Vector2d(-10, -54), Math.toRadians(-90))
                .build();

        goOverLaunchLine = drive.trajectoryBuilder(dropOffSecondWobbleGoal.end(), Math.toRadians(90))
//                .addTemporalMarker(1, wobble::armMiddle)
                .splineToConstantHeading(new Vector2d(-6, -39), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, -34), Math.toRadians(-90))
                .build();
    }

    public void run() throws InterruptedException {
        runtime.reset();

        next(State.GO_TO_SHOOTING_POSITION);
        drive.followTrajectoryAsync(launchPosition);

//        int push = 0;

        //loop
        while (op.opModeIsActive()) {
            double elapsed = runtime.seconds() - time;
            switch (currentState) {
                case GO_TO_SHOOTING_POSITION:
                    if (useShooter) {
                        double shootTime = 0.3;
                        if (elapsed < shootTime + 0.6) {}
                        else if (elapsed < shootTime + 0.7) {
                            indexer.setPower(1);
                            intake.setPower(1);
                            shooter.activate();
                            shooter.allow();
                        } else if (elapsed < shootTime + 0.9) {
                            shooter.push();
                        } else if (elapsed < shootTime + 1.9) {
                            shooter.release();
                        } else if (elapsed < shootTime + 2.1) {
                            shooter.push();
                        } else if (elapsed < shootTime + 3.1) {
                            shooter.release();
                        }
//                         else if (elapsed < shootTime + 3.3) {
//                            shooter.push();
//                        } else if (elapsed < shootTime + 4.3) {
//                            shooter.release();
//                        }
                    }
                    if (!drive.isBusy() && elapsed > 0.3 + 0.9 + 2) {
                        drive.turnAsync(Math.toRadians(20));
                        flap.setPosition(0.205);
                        next(State.ACTION_SHOOT_THREE_RINGS);
                    }
                    break;
                case ACTION_SHOOT_THREE_RINGS:
                    if (useShooter) {
                        double shootTime = 0;
                        if (elapsed < shootTime) {}
                        else if (elapsed < shootTime + 0.2) {
                            shooter.push();
                        } else if (elapsed < shootTime + 1.4) {
                            shooter.release();
                        } else if (elapsed < shootTime + 1.6) {
                            shooter.push();
                        } else if (elapsed < shootTime + 1.8) {
                            shooter.release();
                        }
//                        } else if (elapsed < shootTime + 2) {
//                            shooter.push();
//                        } else if (elapsed < shootTime + 3) {
//                            shooter.release();
//                        }
//                         else if (elapsed < shootTime + 3.2) {
//                            shooter.push();
//                        } else if (elapsed < shootTime + 4.2) {
//                            shooter.release();
//                        }
                        else {
                            shooter.deactivate();
                            intake.setPower(0);
                            indexer.setPower(0);
                            wobble.armDown();
                            //wobble.loosen();
                            next(State.GO_TO_WOBBLE_GOAL);
                            drive.followTrajectoryAsync(dropOffWobbleGoal);
                        }
                    } else {
                        if (elapsed > 1) {
                            next(State.GO_TO_WOBBLE_GOAL);
                            wobble.armDown();
                            //wobble.loosen();
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
//                    if (elapsed < 0.3) {
//
//                    } else {
                    drive.followTrajectoryAsync(pickUp3RingsIntermediatePoint);
                    next(State.GO_TO_3_RINGS);
                    //}
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
                    if (drive.isBusy() && elapsed > 2.5) {
                        intake.setPower(-1);
                    }
                    if (!drive.isBusy()) {
                        intake.setPower(0);
                        indexer.setPower(0);
//                        next(State.IDLE);
                        drive.followTrajectoryAsync(getInPositionForSecondWobbleGoal);
                        next(State.ALIGN_WOBBLE_GOAL);
                    }
                    break;
                case ALIGN_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        //next(State.PICK_UP_WOBBLE_2);
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
                    if (elapsed < 0.3) {
                        wobble.grip();
                    } else {
                        drive.followTrajectoryAsync(shoot2MoreRings);
                        next(State.GO_TO_SHOOTING_POSITION_2);
//                        next(State.IDLE);
                    }
                    break;
                case GO_TO_SHOOTING_POSITION_2:
//                    if (useShooter) {
//                        flap.setPosition(0.1845);
//                        double shootTime = 0.3;
//                        if (elapsed < shootTime + 0.6) {
//                        } else if (elapsed < shootTime + 0.7) {
//                            indexer.setPower(1);
//                            intake.setPower(1);
//                            shooter.activate();
//                            shooter.allow();
//                        } else if (elapsed < shootTime + 0.9) {
//                            shooter.push();
//                        } else if (elapsed < shootTime + 1.9) {
//                            shooter.release();
//                        } else if (elapsed < shootTime + 2.1) {
//                            shooter.push();
//                        } else if (elapsed < shootTime + 3.1) {
//                            shooter.release();
//                        } else if (elapsed < shootTime + 3.3) {
//                            shooter.push();
//                        } else if (elapsed < shootTime + 4.3) {
//                            shooter.release();
//                        } else {
//                            shooter.deactivate();
//                            intake.setPower(0);
//                            indexer.setPower(0);
//                            wobble.armDown();
//                            drive.followTrajectoryAsync(dropOffSecondWobbleGoal);
//                            next(State.DROP_OFF_SECOND_WOBBLE_GOAL);
//                        }
//                    } else {
//                        if (elapsed > 3.4) {
                            drive.followTrajectoryAsync(dropOffSecondWobbleGoal);
                            next(State.DROP_OFF_SECOND_WOBBLE_GOAL);
//                        }
//                    }
                    break;
                case DROP_OFF_SECOND_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        next(State.ACTION_DROP_OFF_SECOND_WOBBLE_GOAL);
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
//            telemetry.addData("push", push);
            telemetry.update();
        }
    }
}