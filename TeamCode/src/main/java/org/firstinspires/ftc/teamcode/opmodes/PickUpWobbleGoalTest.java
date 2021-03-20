package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Wobble;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Pick Up Wobble Goal")
public class PickUpWobbleGoalTest extends LinearOpMode {

    enum State {
        // trajectories
        DRIVE_TO_WOBBLE_GOAL,
        LEAVE_WOBBLE_GOAL,

        // motor movements, stationary
        ACTION_WOBBLE_GOAL,
        IDLE,
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Wobble wobble = new Wobble(hardwareMap);
        Pose2d startingPosition = new Pose2d(-48, -41, Math.toRadians(90));
        drive.setPoseEstimate(startingPosition);

        Trajectory driveToWobbleGoal = drive.trajectoryBuilder(startingPosition)
                .splineToConstantHeading(new Vector2d(-48,-28), Math.toRadians(90))
                .build();

        Trajectory leaveWobbleGoal = drive.trajectoryBuilder(driveToWobbleGoal.end())
                .splineTo(new Vector2d(-30, -28), Math.toRadians(-20))
//                .splineTo(new Vector2d(-8, -54), Math.toRadians(-90))
                .build();

        ElapsedTime runtime = new ElapsedTime();
        wobble.release();
        wobble.armDown();

        waitForStart();
        runtime.reset();

        double endTime = runtime.seconds();
        State currentState = State.DRIVE_TO_WOBBLE_GOAL;
        drive.followTrajectoryAsync(driveToWobbleGoal);


        while (opModeIsActive()) {
            switch (currentState) {
                case DRIVE_TO_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        endTime = runtime.seconds();
                        currentState = State.ACTION_WOBBLE_GOAL;
                    }
                    break;
                case ACTION_WOBBLE_GOAL:
                    if (runtime.seconds() - endTime < 0.5) {
                        wobble.grip();
                    } else if (runtime.seconds() - endTime < 1) {
                        wobble.armUp();
                    } else {
                        drive.followTrajectoryAsync(leaveWobbleGoal);
                        endTime = runtime.seconds();
                        currentState = State.LEAVE_WOBBLE_GOAL;
                    }
                    break;
//                case LEAVE_WOBBLE_GOAL:
//                    if (!drive.isBusy()) {
//                        currentState = State.IDLE;
//                    }
//                    break;
            }
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("state", currentState);
            telemetry.update();
        }
    }
}
