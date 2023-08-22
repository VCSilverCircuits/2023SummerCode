package vcsc.teamcode.Chassis;

import static vcsc.teamcode.Chassis.DriveConstants.TRACK_WIDTH;
import static vcsc.teamcode.Chassis.DriveConstants.encoderTicksToInches;
import static vcsc.teamcode.Chassis.DriveConstants.kA;
import static vcsc.teamcode.Chassis.DriveConstants.kStatic;
import static vcsc.teamcode.Chassis.DriveConstants.kV;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class DriveHardware extends MecanumDrive {

    //create motor objects
    private final DcMotor leftFront, leftRear, rightRear, rightFront;
    //create IMU object
    private final IMU imu;
    //PID vars and object
    public static double rotationP = 0.04, rotationD = 0.005, rotationI = 0;
    PIDFController PivotPID;
    //vars for odo and an array to store the motors

    public static double LATERAL_MULTIPLIER = 1.2;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public double roundedError;
    public double error;
    public double directionAdjusted;
    public double targetAngle;
    public double directionAdjusted360;
    public double headingAdjusted360;
    double RPM;
    double TPS;
    double tics;
    double velocity, acceleration;
    double velocityOld, timeDelta;
//    private final List<DcMotorEx> motors;
    final FtcDashboard dashboard = FtcDashboard.getInstance();

    //make a timer
    ElapsedTime holdTimer = new ElapsedTime();

    //make some variables needed
    private boolean headingAccount;
    private double desiredHeading;
    double rotation;
    double speed;
    int count = 0;
    double lf2 = 0;
    double rf2 = 0;
    double lr2 = 0;
    double rr2 = 0;
    double savedHeading;
    double lastRotation;
    int timesZeroIsCrossed = 0;

    double savedDirection;
    int timesZeroIsCrossedDirection = 0;

    //hardware map all motors adn IMU needed
    public DriveHardware(final HardwareMap hMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        leftFront = hMap.get(DcMotorEx.class, "frontleft");
        leftRear = hMap.get(DcMotorEx.class, "rearleft");
        rightRear = hMap.get(DcMotorEx.class, "rearright");
        rightFront = hMap.get(DcMotorEx.class, "frontright");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        setLocalizer(new ThreeWheelOdo(hMap));
//        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);


        //set up IMU
        imu = hMap.get(IMU.class, "IMU");

        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationI);
    }




    public void dpadDrive(boolean up, boolean down){
        if (up){
            rightFront.setPower(1);
            rightRear.setPower(1);
            leftFront.setPower(1);
            leftRear.setPower(1);
        }else if (down){
            rightFront.setPower(-1);
            rightRear.setPower(-1);
            leftFront.setPower(-1);
            leftRear.setPower(-1);
        }else {
            rightFront.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            leftRear.setPower(0);
        }
    }

    //field centric drive method
    public void FieldCentricTrig(double left_stick_y, double right_stick_x, double left_stick_x, boolean left_bumper, boolean aButton) {

        //Get current heading from IMU.
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = (((angles.firstAngle) % 360));//Get current heading.

        //Math for field centric driving
        final double direction = (Math.atan2(-left_stick_y, -left_stick_x));//Find the direction we want the robot to travel.
        final double direction2 = direction + Math.toRadians(heading);//Add heading for field centric
        speed = Math.min(1.0, sqrt(-left_stick_y * -left_stick_y + -left_stick_x * -left_stick_x));//Math to get the speed at witch we need to drive
        double power1 = speed * Math.sin(direction2 - Math.PI / 4.0);//Powers for the different corners of the drivetrain.
        double power2 = speed * Math.sin(direction2 + Math.PI / 4.0);

        PivotPID.setPIDF(rotationP, rotationI, rotationD, 0);

        if (savedHeading > (heading + 300)){
            timesZeroIsCrossed ++;
            savedHeading = heading;
        }else if (savedHeading < (heading - 300)){
            timesZeroIsCrossed --;
            savedHeading = heading;
        }else {
            savedHeading = heading;
        }
        double fixedHeading = angleAdd(heading, timesZeroIsCrossed);


        //logic for correcting drift
        if (right_stick_x == 0) {
            rotation = PivotPID.calculate(fixedHeading, desiredHeading);
            if (headingAccount) {
                desiredHeading = fixedHeading;
                headingAccount = false;
            }

        } else {
            headingAccount = true;
            rotation = pow(-right_stick_x, 3.0);
        }

        final double lf = power1 - (rotation / 1.5);
        final double rf = power2 + (rotation / 1.5);
        final double lr = power2 - (rotation / 1.5);
        final double rr = power1 + (rotation / 1.5);
//        //logic for making the robot speedup and slow down
//        double speedOfRamp = 0.0001;
//        if ((count == 0) && (holdTimer.milliseconds() > 75)) {
//            if (lf > lf2) {
//                lf2 = lf + speedOfRamp;
//            } else if (lf < lf2) {
//                lf2 = lf - speedOfRamp;
//            }
//            if (rf > rf2) {
//                rf2 = rf + speedOfRamp;
//            } else if (rf < rf2) {
//                rf2 = rf - speedOfRamp;
//            }
//            if (lr > lr2) {
//                lr2 = lr + speedOfRamp;
//            } else if (lr < lr2) {
//                lr2 = lr - speedOfRamp;
//            }
//            if (rr > rr2) {
//                rr2 = rr + speedOfRamp;
//            } else if (rr < rr2) {
//                rr2 = rr - speedOfRamp;
//            }
//            if ((rr == rr2) && (lf == lf2) && (lr == lr2) && (rf == rf2)) {
//                count = 1;
//            }
//            holdTimer.reset();
//        } else if ((rr != rr2) && (lf != lf2) && (lr != lr2) && (rf != rf2)) {
//            count = 0;
//        }
        //set motor powers and if the right bumper is pressed then double speed.
        rightFront.setPower(rf);
        rightRear.setPower(rr);
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        if (aButton){
            savedHeading = heading;
            imu.resetYaw();
        }
    }


    //field centric drive method with auto rotating to optimal heading
    public double FieldCentricTrigHadingAdjust(double left_stick_y, double right_stick_x, double left_stick_x, boolean left_bumper, boolean aButton) {

        //Get current heading from IMU.
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = ((angles.firstAngle) % 360);//Get current heading.

        //Math for field centric driving
        final double direction = (Math.atan2(-left_stick_y, -left_stick_x));//Find the direction we want the robot to travel.
        final double direction2 = direction + Math.toRadians(heading);//Add heading for field centric
        speed = Math.min(1.0, sqrt(-left_stick_y * -left_stick_y + -left_stick_x * -left_stick_x));//Math to get the speed at witch we need to drive
        double power1 = speed * Math.sin(direction2 - Math.PI / 4.0);//Powers for the different corners of the drivetrain.
        double power2 = speed * Math.sin(direction2 + Math.PI / 4.0);

        directionAdjusted = (Math.atan2(-left_stick_x, -left_stick_y));//joystick input adjusted to give outputs the same as heading.
        directionAdjusted360 = angleWrapDegrees(Range.scale(Math.toDegrees(directionAdjusted), 180, -180, 0, 360) + 180);
        headingAdjusted360 = angleWrapDegrees(Range.scale(heading, 180, -180, 0, 360) + 180);


//        error = Math.toDegrees(angleWrap(directionAdjusted - Math.toRadians(heading)));
        error = angleWrapDegrees(directionAdjusted360 - headingAdjusted360);
        roundedError = Math.round(error / 90) * 90;

        PivotPID.setPIDF(rotationP, rotationI, rotationD, 0);

        if (savedHeading > (headingAdjusted360 + 300)){
            timesZeroIsCrossed ++;
            savedHeading = headingAdjusted360;
        }else if (savedHeading < (headingAdjusted360 - 300)){
            timesZeroIsCrossed --;
            savedHeading = headingAdjusted360;
        }else {
            savedHeading = headingAdjusted360;
        }
        double fixedHeading = angleAdd(headingAdjusted360, timesZeroIsCrossed);

//        if (savedDirection > (Math.toDegrees(directionAdjusted) + 170)){
//            timesZeroIsCrossedDirection ++;
//            savedDirection = Math.toDegrees(directionAdjusted);
//        }else if (savedDirection < (Math.toDegrees(directionAdjusted) - 170)){
//            timesZeroIsCrossedDirection --;
//            savedDirection = Math.toDegrees(directionAdjusted);
//        }else {
//            savedDirection = Math.toDegrees(directionAdjusted);
//        }
        if (savedDirection > (directionAdjusted360 + 300)){
            timesZeroIsCrossedDirection ++;//++
            savedDirection = directionAdjusted360;
        }else if (savedDirection < (directionAdjusted360 - 300)){
            timesZeroIsCrossedDirection --;//--
            savedDirection = directionAdjusted360;
        }else {
            savedDirection = directionAdjusted360;//timesZeroIsCrossedDirection +
        }
        if (((left_stick_x < 0.15) && (left_stick_x > -0.15)) && ((left_stick_y < 0.15) && (left_stick_y > -0.15))){
            timesZeroIsCrossedDirection = 0;
        }
        targetAngle = angleAdd(directionAdjusted360, timesZeroIsCrossedDirection + timesZeroIsCrossed);//angleAdd(Math.toDegrees(angleWrap(directionAdjusted + Math.toRadians(-roundedError))), timesZeroIsCrossed);

//        //logic for correcting drift
//        if (right_stick_x == 0) {
//            if ((left_stick_x < 0.15) && (left_stick_y < 0.15)){
//                rotation = PivotPID.calculate(fixedHeading, lastRotation);//desiredHeading
//            }else {
                rotation = -PivotPID.calculate(fixedHeading, targetAngle);//desiredHeading
//                lastRotation = rotation;
//            }
//            if (headingAccount) {
//                desiredHeading = fixedHeading;
//                headingAccount = false;
//            }
//
//        } else {
//            headingAccount = true;
//            rotation = pow(-right_stick_x, 3.0);
//        }

        final double lf = power1 - (rotation / 1.5);
        final double rf = power2 + (rotation / 1.5);
        final double lr = power2 - (rotation / 1.5);
        final double rr = power1 + (rotation / 1.5);
        //set motor powers and if the right bumper is pressed then double speed.
        rightFront.setPower(rf);
        rightRear.setPower(rr);
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        if (aButton){
            savedHeading = heading;
            imu.resetYaw();
        }
        return heading;
    }








    //fix angle so it will not jump
    public double angleAdd(double angle, int timeZeroIsCrossed){
        return angle + (timeZeroIsCrossed * 360);
    }

    // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

    public double angleWrapDegrees(double degrees){
        while (degrees > 360 || degrees < 0) {
            if (degrees > 360) {
                degrees = degrees - 360;
            } else {
                degrees = degrees + 360;
            }
        }
        return degrees;
    }





    //some trash about odo that's dumb
    @Override
    protected double getRawExternalHeading() {
        return 0;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
//        for (DcMotorEx motor : motors) {
//            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
//        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public double getRPM(){
        return RPM;//((rightFront.getCurrentPosition() / 28.00) / 14.8);
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void updateSensors(double timeDelta) {
        TPS = ((rightFront.getCurrentPosition() - tics) / 28) / timeDelta * 1000;
        RPM = (TPS * 60) / 14.8;
        tics = rightFront.getCurrentPosition();

        velocityOld = velocity;
        velocity = getRPM() * 0.301 / 60;
        acceleration = (velocity - velocityOld) / timeDelta * 1000;
    }

    public void update() {
        timeDelta = holdTimer.milliseconds();
        updateSensors(timeDelta);
        holdTimer.reset();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();
        updatePoseEstimate();
        drawRobot(field, getPoseEstimate());
        dashboard.sendTelemetryPacket(packet);
    }

    //method to draw robot on dashboard
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}
