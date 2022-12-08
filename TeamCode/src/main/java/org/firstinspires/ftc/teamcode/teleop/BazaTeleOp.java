package org.firstinspires.ftc.teamcode.teleop;

//import necessary ftc packages
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BazaTeleOp", group = "TeleOp")
public class BazaTeleOp extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public HardwareTeleOp robot = new HardwareTeleOp();

    double x;
    double t = 0;

    double joyScale = 1;

    public void init() {
        robot.initialize(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    public void init_loop() {
    }

    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if(gamepad1.back) {
            robot.rrmode = true;
        }
        if(gamepad1.start) {
            joyScale = 1;
        }
        if(gamepad1.back){
            joyScale = 0.5;
        }
        if(gamepad1.left_trigger> 0.5){
            if(gamepad1.right_trigger> 0.5){
                robot.rrswitch(!robot.rrmode);
            }
        }
        if(robot.rrmode) {
            if (gamepad2.left_trigger > 0.5) {
                robot.openGripper();
                telemetry.addData("Gripper", robot.gripper.getPosition());
            } else if (gamepad2.right_trigger > 0.5) {
                robot.closeGripper();
                telemetry.addData("Gripper", robot.gripper.getPosition());
            }
//            telemetry.addData("pos-lift1", robot.lift_left.getCurrentPosition());
//            telemetry.addData("pos-lift2", robot.lift_right.getCurrentPosition());
//            telemetry.addData("target-lift1", robot.lift_left.getTargetPosition());
//            telemetry.addData("target-lift2", robot.lift_right.getTargetPosition());
//            telemetry.addData("height", robot.robot_height);
            if (gamepad2.a){
                robot.pickup_fata();
            }
            if(gamepad2.y){
                robot.cone_up_high_fata();
            }
            if(gamepad2.x){
                robot.cone_up_mid_fata();
            }
            if(gamepad2.b){
                robot.cone_up_low_fata();
            }
        }
        if (robot.chassis_test && robot.rrmode) {

            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;
            double[] speeds = {
                    (drive + strafe + twist),
                    (drive + strafe - twist),
                    (drive - strafe + twist),
                    (drive - strafe - twist)
            };
            // Because we are adding vectors and motors only take values between
            // [-1,1] we may need to normalize them.

            // Loop through all values in the speeds[] array and find the greatest
            // magnitude.  Not the greatest velocity.

            double max = Math.abs(speeds[0]);
            for (double speed : speeds) {
                if (max < Math.abs(speed)) max = Math.abs(speed);
            }

            // If and only if the maximum is outside of the range we want it to be,
            // normalize all the other speeds based on the given speed value.
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            // apply the calculated values to the motors.
            robot.leftFront.setPower(speeds[0] * joyScale);
            robot.rightFront.setPower(speeds[1] * joyScale);
            robot.leftRear.setPower(speeds[2] * joyScale);
            robot.rightRear.setPower(speeds[3] * joyScale);
        }
    }
}