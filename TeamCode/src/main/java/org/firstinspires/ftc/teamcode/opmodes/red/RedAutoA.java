package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RedAutoA")
public class RedAutoA extends LinearOpMode {

    enum State {
        DROP_OFF_WOBBLE_GOAL,
        DROP_OFF_WOBBLE_GOAL_2,
        DROP_OFF_WOBBLE_GOAL_3,
        //SHOOT_POWERSHOTS,
        SHOOT_HIGH_GOAL,
        SECOND_WOBBLE_GOAL_1,
        SECOND_WOBBLE_GOAL_2,
        DROP_OFF_SECOND_WOBBLE_GOAL,
        IDLE,
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // We define the current state we're on
        // Default to IDLE
        State currentState = State.IDLE;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //starting position for robot - halfway across first tile
        Pose2d startingPosition = new Pose2d(-63, -54, Math.toRadians(0)); //maximum starting position
        drive.setPoseEstimate(startingPosition);

        //trajectory to go to tile a to drop off wobble goal
        //Pose2d positionForA = new Pose2d(-54, -54, Math.toRadians(-90)); //new position if we have to run trajectory a, robot rotates -90 so we can strafe
        Trajectory dropOffWobbleGoal = drive.trajectoryBuilder(startingPosition)
                .splineTo(new Vector2d(-8,-54),Math.toRadians(0))
                .build();

        double turnAngle = Math.toRadians(-90);

        //getting to position to shoot powershots
        /*Trajectory shootPowershots = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                //rerotating as robot moves to spot to shoot powershots,
                //x = -12 to be safe so that bot doesn't go over launch line
                .lineTo(new Vector2d(11,42)) //we could also turn and then strafe (strafeLeft or strafeRight)
                .build();*/

        Trajectory shootHighGoal = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .lineTo(new Vector2d(3,-36))
                .build();

        //trajectory to get rings from stack
        /*Trajectory pickUpRings = drive.trajectoryBuilder(shootHighGoal.end())
                .lineTo(new Vector2d(-18,0))
                .build();*/

        //trajectories to pick up 2nd wobble goal
        Trajectory secondWobbleGoalOne = drive.trajectoryBuilder(shootHighGoal.end())
                .lineTo(new Vector2d(-48,-36))
                .build();

        Trajectory secondWobbleGoalSecond = drive.trajectoryBuilder(secondWobbleGoalOne.end())
                .strafeLeft(3) //.strafeTo(new Vector2d(0,3))
                .build();

        //trajectory to drop off second wobble goal
        Trajectory dropOffSecondWobbleGoal = drive.trajectoryBuilder(secondWobbleGoalSecond.end())
                .strafeTo(new Vector2d(40,-24))
                .build();


        waitForStart();

        currentState = State.DROP_OFF_WOBBLE_GOAL;
        drive.followTrajectoryAsync(dropOffWobbleGoal);



        while (opModeIsActive()) {
            switch (currentState) {

                case DROP_OFF_WOBBLE_GOAL:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        //currentState = State.SHOOT_POWERSHOTS;
                        //drive.followTrajectoryAsync(shootPowershots);
                        currentState = State.DROP_OFF_WOBBLE_GOAL_2;
                        drive.turnAsync(turnAngle);

//                        currentState = State.SHOOT_HIGH_GOAL;
//                        turnAngle = Math.toRadians(90);
//                        drive.turnAsync(turnAngle);
//                        drive.followTrajectoryAsync(shootHighGoal);
                    }
                    break;
                case DROP_OFF_WOBBLE_GOAL_2:
                    if (!drive.isBusy()){
                        currentState = State.DROP_OFF_WOBBLE_GOAL_3;
                        turnAngle = Math.toRadians(90);
                        drive.turnAsync(turnAngle);
                    }
                    break;
                case DROP_OFF_WOBBLE_GOAL_3:
                    if (!drive.isBusy()){
                        currentState = State.SHOOT_HIGH_GOAL;
                        drive.followTrajectoryAsync(shootHighGoal);
                    }
                case SHOOT_HIGH_GOAL:
                    if (!drive.isBusy()) {
                        currentState = State.SECOND_WOBBLE_GOAL_1;
                        drive.followTrajectoryAsync(secondWobbleGoalOne);
//                        turnAngle = Math.toRadians(180);
//                        drive.turnAsync(turnAngle);
                    }
                    break;
                case SECOND_WOBBLE_GOAL_1:
                    if (!drive.isBusy()) {
//                        currentState = State.SECOND_WOBBLE_GOAL_2;
//                        drive.followTrajectoryAsync(secondWobbleGoalOne);
                    }
                    break;
                case SECOND_WOBBLE_GOAL_2:
                    if (!drive.isBusy()) {
//                        currentState = State.DROP_OFF_SECOND_WOBBLE_GOAL;
//                        turnAngle = Math.toRadians(-90);
//                        drive.turn(turnAngle);
//                        drive.followTrajectoryAsync(dropOffSecondWobbleGoal);
                    }
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
