package vcsc.teamcode.Chassis;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import vcsc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;


@Config
public class ThreeWheelOdo extends ThreeTrackingWheelLocalizer {

    //variables for 3 wheel odo
    public static double TICKS_PER_REV = 8192;//rev encoder tics
    public static double WHEEL_RADIUS = 0.6889764; // in in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 6; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;

    public ThreeWheelOdo(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // perpendicular
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rearright"));  //left
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontleft")); //right
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rearleft")); //perpendicular

        // reverse encoder directions
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    //convert tics to inches
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
