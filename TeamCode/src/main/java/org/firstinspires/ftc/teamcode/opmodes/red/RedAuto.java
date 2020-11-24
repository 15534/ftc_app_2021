package org.firstinspires.ftc.teamcode.opmodes.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RedAuto")
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int ringCount = 0;

        //determining which program to run
        if (ringCount == 0) {
            //a-tile first
            RedAutoA auton = new RedAutoA();
            auton.runOpMode();
        } else if (ringCount == 1){
            //b-tile first
            RedAutoB auton = new RedAutoB();
            auton.runOpMode();
        } else if (ringCount == 4) {
            //c-tile first
            RedAutoC auton = new RedAutoC();
            auton.runOpMode();
        }
    }
}
/* code that will get run after dropping off wobble goal in A
        Pose2d positionAfterFirstMovement = trajectoryForwardA.end().plus(new Pose2d (0,0, turnAngle));
        Trajectory trajectoryForwardATwo = drive.trajectoryBuilder(positionAfterFirstMovement)
                .lineToConstantHeading(new Vector2d(0, 0))
                .build();
         */

        /*
        //trajectory to go to tile b to drop off wobble goal
        Trajectory trajectoryForwardB = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(24, -36)) //maybe need to change lineTo for this
                .build();

        //trajectory to go to tile b to drop off wobble goal
        Trajectory trajectoryForwardC = drive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(48, -60))
                .build();
        */