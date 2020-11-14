package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1440;
    public static double WHEEL_RADIUS = 1.14173; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double LATERAL_DISTANCE = 13.587; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6.184; // in; offset of the lateral wheel
    public static double X_MULTIPLIER = 100/97.6807; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 100/95.310; // Multiplier in the Y direction
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE/2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE/2, 0), // right
                new Pose2d(-FORWARD_OFFSET, 0, Math.toRadians(90)) // rear
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front_left"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front_right"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rear_left"));
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double encoderTicksToInchesRear(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV / (40.0/24);
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition())*X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition())*X_MULTIPLIER,
                encoderTicksToInchesRear(frontEncoder.getCurrentPosition())*Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()),
                encoderTicksToInches(rightEncoder.getRawVelocity()),
                encoderTicksToInches(frontEncoder.getRawVelocity())
        );
    }
}
