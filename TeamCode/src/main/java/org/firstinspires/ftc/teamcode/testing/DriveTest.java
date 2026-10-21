package org.firstinspires.ftc.teamcode.testing;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "Eric's Training Session")
public class DriveTest extends OpMode {
    private Follower follower;
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }
    @Override
    public void loop() {

        //Teleop with Auto Hold
        ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();
    }
}