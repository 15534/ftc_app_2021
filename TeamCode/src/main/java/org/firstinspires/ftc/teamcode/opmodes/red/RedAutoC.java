package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RedAutoC")
public class RedAutoC extends LinearOpMode {

    enum State {
        DROP_OFF_WOBBLE_GOAL,
        DROP_OFF_WOBBLE_GOAL_2,
        //SHOOT_POWERSHOTS,
        SHOOT_HIGH_GOAL,
        SHOOT_HIGH_GOAL_2,
        PICK_UP_RINGS,
        PICK_UP_SECOND_WOBBLE_1,
        PICK_UP_SECOND_WOBBLE_2,
        PICK_UP_SECOND_WOBBLE_3,
        DROP_OFF_SECOND_WOBBLE_GOAL,
        SHOOT_SECOND_HIGH_GOAL,
        GET_TO_LAUNCH_LINE,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //constants
        State currentState = State.IDLE;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //starting position for robot - halfway across first tile
        Pose2d startingPosition = new Pose2d(-54, -54, Math.toRadians(0)); //maximum starting position
        drive.setPoseEstimate(startingPosition);

        //wobble goal trajectory
        Trajectory dropOffWobbleGoal = drive.trajectoryBuilder(startingPosition)
                .splineTo(new Vector2d(42,-54), Math.toRadians(0))
                .build();

        Trajectory dropOffWobbleGoalTwo = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .splineTo(new Vector2d(3,-36), Math.toRadians(0)) // (3,-39) ??
                .build();

        double turnAngle = Math.toRadians(-90);
        //turn angle for dropping off wobble goal
        //shoot powershots trajectory
        /*Trajectory shootPowershots = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .lineTo(new Vector2d(39,42))
                .build();*/

//        Trajectory shootHighGoal = drive.trajectoryBuilder(dropOffWobbleGoalTwo.end())
//                .lineTo(new Vector2d(3,-36))
//                .build();

        //trajectory for turning around and picking up 3/4 rings
        Trajectory pickUpRings = drive.trajectoryBuilder(dropOffWobbleGoalTwo.end())
                .splineTo(new Vector2d(-15,-36), Math.toRadians(180))
                .build();

        //trajectory for moving to [-48,0] for first part of getting second wobble goal
        Trajectory pickUpSecondWobbleOne = drive.trajectoryBuilder(pickUpRings.end())
                .lineTo(new Vector2d(-48,0))
                .build();

        //trajectory for getting to second wobble goal for pickup - NOTE: may not need to strafe so much b/c we might hit and knock the wobble goal over
        Trajectory pickUpSecondWobbleTwo = drive.trajectoryBuilder(pickUpSecondWobbleOne.end())
                .strafeRight(15)
                .build();

        //getting into a position to drop off second wobble goal
        Trajectory dropOffSecondWobbleGoal = drive.trajectoryBuilder(pickUpSecondWobbleTwo.end())
                .strafeTo(new Vector2d(-57,-15))
                .build();

        //strafting right to be in perfect position for dropoff
        Trajectory dropOffSecondWobbleGoal2 = drive.trajectoryBuilder(dropOffSecondWobbleGoal.end())
                .strafeRight(24)
                .build();

        //getting back to launch line to shoot sum more high rings
        Trajectory shootHighRingsTwo = drive.trajectoryBuilder(dropOffSecondWobbleGoal2.end())
                .lineTo(new Vector2d(3,-36))
                .build();

        //getting to over launch line
        Trajectory movingToLaunchLine = drive.trajectoryBuilder(shootHighRingsTwo.end())
                .lineTo(new Vector2d(9,0))
                .build();

        waitForStart();

        currentState = State.DROP_OFF_WOBBLE_GOAL;
        drive.followTrajectoryAsync(dropOffWobbleGoal); //getting to (42,-54) to drop off wobble goal

        //loop
        while (opModeIsActive()) {
            switch (currentState) {
                case DROP_OFF_WOBBLE_GOAL:
                    if(!drive.isBusy()){
                        currentState = State.DROP_OFF_WOBBLE_GOAL_2;
//                        turnAngle = Math.toRadians(-90);
                        drive.turnAsync(turnAngle);
                        //and then put wobble goal down...
                    }
                    break;
                case DROP_OFF_WOBBLE_GOAL_2:
                    if(!drive.isBusy()){
                        currentState = State.PICK_UP_RINGS;
                        drive.followTrajectoryAsync(dropOffWobbleGoalTwo);
                    }
                    break;
                case SHOOT_HIGH_GOAL:
                    if (!drive.isBusy()) {
                        currentState = State.SHOOT_HIGH_GOAL_2;
                        drive.followTrajectoryAsync(pickUpRings);
                    }
                    break;
                case SHOOT_HIGH_GOAL_2:
                    if(!drive.isBusy()){
                        currentState = State.PICK_UP_RINGS;
                        turnAngle = Math.toRadians(90);
                        drive.turnAsync(turnAngle);
                        //and then shoot into high goal...
                    }
                    break;

                case PICK_UP_RINGS:
                    if (!drive.isBusy()) {
                        currentState = State.PICK_UP_SECOND_WOBBLE_1;
                        drive.followTrajectoryAsync(pickUpRings);
                    }
                    break;
                case PICK_UP_SECOND_WOBBLE_1:
                    if (!drive.isBusy()){
                        currentState = State.PICK_UP_SECOND_WOBBLE_2;
                        drive.followTrajectoryAsync(pickUpSecondWobbleTwo);
                    }
                    break;
                case PICK_UP_SECOND_WOBBLE_2:
                    if(!drive.isBusy()){
                        currentState = State.PICK_UP_SECOND_WOBBLE_3;
                        drive.followTrajectoryAsync(dropOffSecondWobbleGoal);
                    }
                    break;
                case PICK_UP_SECOND_WOBBLE_3:
                    if (!drive.isBusy()){
                        currentState = State.DROP_OFF_SECOND_WOBBLE_GOAL;
                        drive.followTrajectoryAsync(dropOffSecondWobbleGoal2);
                    }
                    break;
                case DROP_OFF_SECOND_WOBBLE_GOAL:
                    if(!drive.isBusy()){
                        currentState = State.SHOOT_SECOND_HIGH_GOAL;
                        drive.followTrajectoryAsync(shootHighRingsTwo);
                    }
                    break;
                case SHOOT_SECOND_HIGH_GOAL:
                    if (!drive.isBusy()) {
                        turnAngle = Math.toRadians(180);
                        drive.turnAsync(turnAngle);
                    }
                    break;
                case GET_TO_LAUNCH_LINE:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(movingToLaunchLine);
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
