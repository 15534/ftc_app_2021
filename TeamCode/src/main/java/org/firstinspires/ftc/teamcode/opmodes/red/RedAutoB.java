package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="RedAutoB")
public class RedAutoB extends LinearOpMode {

    enum State {
        DROP_OFF_WOBBLE_GOAL,
        //SHOOT_POWERSHOTS,
        SHOOT_HIGH_GOAL,
        SHOOT_HIGH_GOAL_2,
        PICK_UP_RINGS,
        PICK_UP_SECOND_WOBBLE_ONE,
        PICK_UP_SECOND_WOBBLE_TWO,
        PICK_UP_SECOND_WOBBLE_THREE,
        PICK_UP_SECOND_WOBBLE_FOUR,
        DROP_OFF_SEC_WOBBLE_GOAL,
        SHOOT_RINGS_ONE,
        SHOOT_RINGS_TWO,
        RETURNING_ONE,
        RETURNING_TWO,
        RETURNING_THREE,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // We define the current state we're on
        // Default to IDLE
        RedAutoB.State currentState = RedAutoB.State.IDLE;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //starting position for robot - halfway across first tile
        Pose2d startingPosition = new Pose2d(-63, -54, Math.toRadians(0)); //maximum starting position
        drive.setPoseEstimate(startingPosition);

        Trajectory dropOffWobbleGoal = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(36,-54))
                .build();

        /*Trajectory shootPowershots = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .lineTo(new Vector2d(-33,42))
                .build();*/

        //trajectory for shooting into the high goal
        Trajectory shootHighGoal = drive.trajectoryBuilder(dropOffWobbleGoal.end())
                .lineToConstantHeading(new Vector2d(3, -36))
                .build();

        //trajectory for turning around and picking up 1 ring
        Trajectory pickUpRings = drive.trajectoryBuilder(shootHighGoal.end())
                .lineTo(new Vector2d(-15,-36))
                .build();

        //trajectory for moving to [-48,0] for first part of getting second wobble goal
        Trajectory pickUpSecondWobbleOne = drive.trajectoryBuilder(pickUpRings.end())
                .lineTo(new Vector2d(-48,-15))
                .build();

        //trajectory for getting to second wobble goal for pickup - NOTE: may not need to strafe so much b/c we might hit and knock the wobble goal over
//        Trajectory pickUpSecondWobbleTwo = drive.trajectoryBuilder(pickUpSecondWobbleOne.end())
//                .strafeRight(15)
//                .build();

        Trajectory pickUpSecondWobbleThree = drive.trajectoryBuilder(pickUpSecondWobbleOne.end())
                .lineTo(new Vector2d(36,-54))
                .build();

        //no trajectory for pick up wobble goal 4
        //no trajectory for drop wobble goal... yet
        Trajectory shootRingsTwo = drive.trajectoryBuilder(pickUpSecondWobbleThree.end())
                .lineTo(new Vector2d(3,-36))
                .build();

        //no trajectory for returning one
        Trajectory returningTwo = drive.trajectoryBuilder(shootRingsTwo.end())
                .lineTo(new Vector2d(3,-36))
                .build();

        Trajectory returningThree = drive.trajectoryBuilder(returningTwo.end())
                .lineTo(new Vector2d(12,-36))
                .build();

        double turnAngle = Math.toRadians(-90); //turn angle for shooting highshoots
        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        double time = 0.0;

        //currentState = RedAutoB.State.DROP_OFF_WOBBLE_GOAL;
        drive.followTrajectoryAsync(dropOffWobbleGoal);
        currentState = RedAutoB.State.DROP_OFF_WOBBLE_GOAL;

        while (opModeIsActive()) {
            switch (currentState) {
                case DROP_OFF_WOBBLE_GOAL:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.SHOOT_HIGH_GOAL;
                        drive.followTrajectoryAsync(shootHighGoal);
                        time = runtime.seconds(); //getting updated time
                    }
                    break;
                case SHOOT_HIGH_GOAL:
                    if (!drive.isBusy()) {
                        if (runtime.seconds() - time > 3)  {
                            currentState = State.PICK_UP_RINGS;
                            turnAngle = Math.toRadians(180);
                            drive.turnAsync(turnAngle); //after shooting into high goal
                            time = runtime.seconds(); // getting updated time
                            //currentState = RedAutoB.State.PICK_UP_RINGS;
                            //drive.followTrajectoryAsync(pickUpRings);
                        }
                    }
                    break;
                case PICK_UP_RINGS:
                    if (!drive.isBusy()) {
                        if (runtime.seconds() - time > 3) {
                            currentState = State.PICK_UP_SECOND_WOBBLE_TWO;
                            drive.followTrajectoryAsync(pickUpRings);
                            //drive.followTrajectoryAsync(pickUpSecondWobbleOne);
                            time = runtime.seconds();
                        }
                    }
                    break;
//                case PICK_UP_SECOND_WOBBLE_ONE:
//                    if (!drive.isBusy()){
//                        currentState = RedAutoB.State.PICK_UP_SECOND_WOBBLE_TWO;
//                        drive.followTrajectoryAsync(pickUpSecondWobbleTwo);
//                    }
//                    break;
                case PICK_UP_SECOND_WOBBLE_TWO:
                    if (!drive.isBusy()){
                        currentState = RedAutoB.State.PICK_UP_SECOND_WOBBLE_THREE;
                        drive.followTrajectoryAsync(pickUpSecondWobbleThree);
                    }
                    break;
                case PICK_UP_SECOND_WOBBLE_THREE:
                    if (!drive.isBusy()) {
                        turnAngle = Math.toRadians(180);
                        drive.turnAsync(turnAngle);
                        currentState = State.DROP_OFF_SEC_WOBBLE_GOAL;
                    }
                    break;
                case DROP_OFF_SEC_WOBBLE_GOAL:
                    if (!drive.isBusy()) {
                        //drop off the second wobble goal...
                        currentState = State.SHOOT_RINGS_ONE;
                    }
                    break;
                case SHOOT_RINGS_ONE:
                    if (!drive.isBusy()){
                        currentState = RedAutoB.State.SHOOT_RINGS_TWO;
                        drive.followTrajectoryAsync(shootRingsTwo);
                    }
                    break;
                case SHOOT_RINGS_TWO:
                    if (!drive.isBusy()) {
                        //shoot rings...
                        currentState = RedAutoB.State.RETURNING_ONE;
                    }
                    break;
                case RETURNING_ONE:
                    if (!drive.isBusy()){
                        //shoot rings...
                        currentState = RedAutoB.State.RETURNING_TWO;
                        drive.followTrajectoryAsync(returningTwo);
                    }
                    break;
                case RETURNING_TWO:
                    if (!drive.isBusy()){
                        //shoot rings...
                        currentState = RedAutoB.State.RETURNING_THREE;
                        drive.followTrajectoryAsync(returningThree);
                    }
                    break;
                case RETURNING_THREE:
                    if (!drive.isBusy()){
                        //done
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
        }
    }
}
