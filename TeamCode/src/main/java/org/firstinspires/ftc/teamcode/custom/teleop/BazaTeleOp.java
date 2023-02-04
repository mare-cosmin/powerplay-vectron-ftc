package org.firstinspires.ftc.teamcode.custom.teleop;

//import necessary ftc packages
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BazaTeleOp", group = "TeleOp")
public class BazaTeleOp extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public TeleOpAdaptedMecanumDrive robot = new TeleOpAdaptedMecanumDrive(hardwareMap);
    double joyScale = 0.8;
    boolean up_flag = true;
    boolean down_flag = true;

    double time = runtime.seconds();
    double up_time = 0;
    double down_time = 0;

    private int speed = 3;

    public void init() {
        robot.initialize(hardwareMap);
        if(robot.gripper_test)
            robot.closeGripper();
        telemetry.addData("Status", "Initialized");
    }

    public void init_loop() {
    }

    public void start() {
        runtime.reset();
        if(robot.gripper_test && robot.lift_test && robot.servo_brat_test)
            robot.cone_up_low_fata();
    }

    @Override
    public void loop() {
        if(gamepad1.left_trigger> 0.5){
            if(gamepad1.right_trigger> 0.5){
                robot.rrswitch(!robot.rrmode);
            }
        }
        if(robot.rrmode) {
            if(gamepad1.x){
                robot.pickup_stack(1);
            }
//            if(gamepad1.x){
//                robot.pickup_stack(2);
//            }
//            if(gamepad1.y){
//                robot.pickup_stack(3);
//            }
//            if(gamepad1.b){
//                robot.pickup_stack(4);
//            }

            if (gamepad2.left_trigger > 0.5) {
                robot.openGripper();
                telemetry.addData("Gripper", robot.gripper.getPosition());
            } else if (gamepad2.right_trigger > 0.5) {
                robot.closeGripper();
                telemetry.addData("Gripper", robot.gripper.getPosition());
            }
            if(gamepad2.dpad_down){
                robot.backwards = false;
            }
            if(gamepad2.dpad_up){
                robot.backwards = true;
            }
            if(robot.lift_test) {
                telemetry.addData("pos-lift1", robot.lift_exp.getCurrentPosition());
                telemetry.addData("pos-lift2", robot.lift_ctrl.getCurrentPosition());
                telemetry.addData("target-lift1", robot.lift_exp.getTargetPosition());
                telemetry.addData("target-lift2", robot.lift_ctrl.getTargetPosition());
                telemetry.addData("height", robot.robot_height);
            }
            telemetry.addData("encoderl", robot.leftRear.getCurrentPosition());
            telemetry.addData("powerl", robot.leftRear.getPower());
            telemetry.addData("velol", robot.leftRear.getVelocity());

            telemetry.addData("encoderr", robot.rightRear.getCurrentPosition());
            telemetry.addData("pwoerr", robot.rightRear.getPower());
            telemetry.addData("velor", robot.rightRear.getVelocity());
            if (gamepad2.a){
                robot.pickup_fata();
            }
            if(gamepad2.y){
                if(robot.backwards)
                    robot.cone_up_high_spate();
                else
                    robot.cone_up_high_fata();
            }
            if(gamepad2.x){
                if(robot.backwards)
                    robot.cone_up_mid_spate();
                else
                    robot.cone_up_mid_fata();
            }
            if(gamepad2.b){
                if(robot.backwards)
                    robot.cone_up_low_spate();
                else
                    robot.cone_up_low_fata();
            }
            if(gamepad2.left_bumper){
                robot.cone_down_low_fata();
            }

//            if(gamepad2.dpad_down){
//                robot.terminal = true;
//                robot.pickup_fata();
//            }
            if(gamepad1.right_bumper){
                if(up_time+500 < runtime.milliseconds()) {
                    speed += (speed < 3 ? 1 : 0);
                    up_time = runtime.milliseconds();
                }
            }
            if(gamepad1.left_bumper){
                if(down_time+500 < runtime.milliseconds()){
                    speed -= (speed > 1 ? 1 : 0);
                    down_time = runtime.milliseconds();
                }
            }

        }
        telemetry.addData("speed", speed);
        if (robot.chassis_test && robot.rrmode) {
            switch(speed){
                case 1:
                    joyScale = 0.3;
                    break;
                case 2:
                    joyScale = 0.5;
                    break;
                default:
                    joyScale = 0.8;
            }
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    ).times(joyScale)
            );
        }
    }
}