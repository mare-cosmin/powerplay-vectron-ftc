package org.firstinspires.ftc.teamcode.teleop;

//import necessary ftc packages
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomie.HardwareAutonomie;

@TeleOp(name = "BazaTeleOp", group = "TeleOp")
public class BazaTeleOp extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public HardwareTeleOp robot = new HardwareTeleOp();



    double x;
    double t = 0;

    double joyScale;

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
        if (gamepad1.left_trigger > 0.5){
            robot.openGripper();
            telemetry.addData("Gripper", "Open");
        }else if (gamepad1.right_trigger > 0.5){
            robot.closeGripper();
            telemetry.addData("Gripper", "Closed");
        }
        if(gamepad1.x) {
            robot.motor_brat.setPower(0.5);
            robot.motor_brat2.setPower(0.5);
            telemetry.addData("Brat", "Up");
        }else if(gamepad1.y) {
            robot.motor_brat.setPower(-0.5);
            robot.motor_brat2.setPower(-0.5);
            telemetry.addData("Brat", "Down");
        }else{
            robot.motor_brat.setPower(0);
            robot.motor_brat2.setPower(0);
            telemetry.addData("Brat", "Stopped");
        }

        if(!robot.no_roti_test) {

            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;
            ElapsedTime timpRate = new ElapsedTime();
            ElapsedTime salut = new ElapsedTime();
            /*

             */
            double[] speeds = {
                    (drive - strafe + twist),
                    (drive + strafe - twist),
                    (drive + strafe + twist),
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
            robot.leftFront.setPower(speeds[3] * joyScale);
            robot.rightFront.setPower(speeds[2] * joyScale);
            robot.leftBack.setPower(speeds[1] * joyScale);
            robot.rightBack.setPower(speeds[0] * joyScale);
        }
    }
}

