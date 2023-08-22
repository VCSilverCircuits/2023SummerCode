package vcsc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import vcsc.teamcode.Chassis.DriveHardware;

@Config
@TeleOp(name = "New TeleOp", group = "GG(Get Girls)")
public class TeleOpChassisTest extends OpMode {

    //initialize all subsystems
    private DriveHardware Drive;

    @Override
    public void init() {

        Drive = new DriveHardware(hardwareMap);
        //timer for tracking button presses
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        //send telemetry to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //wait for the driver to press start
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    //loop while program is in the int phase
    @Override
    public void init_loop() {

    }

    //while the OpMode Is Active run this code
    @Override
    public void loop() {
//        Drive.FieldCentricTrig(gamepad1.left_stick_y, gamepad1.right_stick_x,
//                gamepad1.left_stick_x, gamepad1.left_bumper, gamepad1.a);
        Drive.dpadDrive(gamepad1.dpad_up, gamepad1.dpad_down);
        telemetry.addData("RPM ", Drive.getRPM());
        telemetry.addData(" velocity", Drive.getVelocity());
        telemetry.addData(" acceleration", Drive.getAcceleration());
        Drive.update();
//        telemetry.addData("joyStick", Drive.directionAdjusted);
//        telemetry.addData("rounded Error", Drive.roundedError);
//        telemetry.addData("error", Drive.error);
//        telemetry.addData("heading 360", Drive.headingAdjusted360);
//        telemetry.addData("direction 360", Drive.directionAdjusted360);
//        telemetry.addData("targetAngle", Drive.targetAngle);//((Math.toDegrees(Drive.directionAdjusted) - 90) * -1));//(Math.toDegrees(Drive.direction) % 360));//direction = (direction % 360);  Range.scale(Math.toDegrees(Drive.direction), -180, 180, 0, 360);
        telemetry.update();
    }//((Math.toDegrees(Drive.direction) - 90) * -1)
    @Override
    public void stop() {

    }
}
