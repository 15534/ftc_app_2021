package org.firstinspires.ftc.teamcode.opmodes.red;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Wobble;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.red.RedA;
import org.firstinspires.ftc.teamcode.opmodes.red.RedB;
import org.firstinspires.ftc.teamcode.opmodes.red.RedC;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(name="RedAuto")
public class RedAuto extends LinearOpMode {

    public static int REG1_CUTOFF = 120;
    public static int REG2_CUTOFF = 120;
    RingPipeline pipeline;
    public DcMotorEx indexer, intake;
    public Servo flap;
    public SampleMecanumDrive drive;
    public Shooter shooter;
    public Pose2d startingPosition = new Pose2d(-63, -57, Math.toRadians(0));
    public Wobble wobble;
    private RedA a;
    private RedB b;
    private RedC c;
    int stack = 1;
    public static boolean useShooter = true;

    @Override
    public void runOpMode() throws InterruptedException {
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        flap = hardwareMap.get(Servo.class, "flap");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        drive = new SampleMecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        wobble = new Wobble(hardwareMap);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flap.setPosition(0.1845);

        telemetry.addData("OPENING CAMERA", "");
        telemetry.update();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        pipeline = new RingPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
//                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }
        });

        PoseStorage.currentPose = startingPosition;

//        shooter.block();
        shooter.allow();
        shooter.release();
        shooter.stickUp();
        wobble.grip();
        sleep(500);
        // TODO enable
        wobble.armUp();


        telemetry.addData("BUILDING TRAJECTORIES (A)", "");
        telemetry.update();
        a = new RedA(this);
        a.buildTrajectories();

        telemetry.addData("BUILDING TRAJECTORIES (B)", "");
        telemetry.update();
        b = new RedB(this);
        b.buildTrajectories();

        telemetry.addData("BUILDING TRAJECTORIES (C)", "");
        telemetry.update();
        c = new RedC(this);
        c.buildTrajectories();


        while (!opModeIsActive() && !isStopRequested()) {
            drive.setPoseEstimate(startingPosition);
            telemetry.addData("READY", "");
            //stack = pipeline.getStack();
            telemetry.addData("stack", stack);
            telemetry.update();
            sleep(50);
        }

        if (opModeIsActive()) {
            // wait for start
            switch (stack) {
                case 0:
                    telemetry.addData("Running A", "");
                    a.run();
                    break;
                case 1:
                    telemetry.addData("Running B", "");
                    b.run();
                    break;
                case 4:
                    telemetry.addData("Running C", "");
                    c.run();
                    break;
            }
        }

    }

    public static class RingPipeline extends OpenCvPipeline {
        static final Point REGION1_A = new Point(115, 130);
        static final Point REGION1_B = new Point(145, 140);

        static final Point REGION2_A = new Point(115, 150);
        static final Point REGION2_B = new Point(145, 160);

        Mat region1_Cb, region2_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        private volatile int avg1, avg2;
        private volatile int stack = 0;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(REGION1_A, REGION1_B));
            region2_Cb = Cb.submat(new Rect(REGION2_A, REGION2_B));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];

            final Scalar BLUE = new Scalar(0, 0, 255);
            final Scalar GREEN = new Scalar(0, 255, 0);

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION1_A, // First point which defines the rectangle
                    REGION1_B, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION2_A, // First point which defines the rectangle
                    REGION2_B, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            if (avg1 < REG1_CUTOFF) {
                stack = 4;
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        REGION1_A, // First point which defines the rectangle
//                        REGION1_B, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        -1);
            } else if (avg2 < REG2_CUTOFF) {
                stack = 1;
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        REGION2_A, // First point which defines the rectangle
//                        REGION2_B, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        -1);
            } else {
                stack = 0;
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        public int getStack()
        {
            return stack;
        }

        public int getAvg1() { return avg1; }
        public int getAvg2() { return avg2; }
    }
}