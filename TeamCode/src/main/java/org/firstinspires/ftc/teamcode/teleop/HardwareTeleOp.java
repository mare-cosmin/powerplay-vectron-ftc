package org.firstinspires.ftc.teamcode.teleop;

//import necessary ftc hardware packages
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareTeleOp {
    //define motors
    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor lift1 = null;
    public DcMotor lift2 = null;

    public boolean rrmode = true;

    public boolean chassis_test = true;
    public boolean lift_test = true;
    public boolean gripper_test = true;
    public boolean servo_brat_test = true;
    public boolean all_in = false;
    public enum Height{
        HIGH,
        MEDIUM,
        LOW
    }

    public Height robot_height = Height.LOW;

    //define servos
    public Servo gripper = null;
    public Servo servo_brat_sus = null;
    public Servo servo_brat_jos = null;

    public void initialize(HardwareMap hardwareMap){
        if(chassis_test || all_in) {
            //initialize motors
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            //set motor directions
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);

            //set motor modes
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //set motor powers
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
        if(gripper_test || all_in) {
            //initialize servos
            gripper = hardwareMap.get(Servo.class, "gripper");
            gripper.setDirection(Servo.Direction.FORWARD);
            gripper.setPosition(0.08);
        }
        if(servo_brat_test || all_in) {
//            servo_brat_jos = hardwareMap.get(Servo.class, "servo_brat_jos");
//            servo_brat_sus = hardwareMap.get(Servo.class, "servo_brat_sus");
//
//            servo_brat_jos.setDirection(Servo.Direction.FORWARD);
//            servo_brat_sus.setDirection(Servo.Direction.FORWARD);
//            servo_brat_jos.setPosition(0);
//            servo_brat_sus.setPosition(0.5);
        }

        if(lift_test || all_in) {
            lift1 = hardwareMap.get(DcMotor.class, "lift_jos");
            lift1.setDirection(DcMotor.Direction.FORWARD);
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setPower(0);

            lift2 = hardwareMap.get(DcMotor.class, "lift_sus");
            lift2.setDirection(DcMotor.Direction.FORWARD);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setPower(0);

            lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public void runToPositionCHASSIS(){
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPowerCHASSIS(double power){

        rightFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setPower(power);
        leftFront.setPower(power);
    }
    public void setPowerZeroCHASSIS(){

        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);
    }
    public void stopAndResetEncodersCHASSIS(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //functii pentru celelalte motoare

    public void closeGripper(){
        gripper.setPosition(1);
    }
    public void openGripper(){
        gripper.setPosition(0);
    }
    public void lift_pos_down(int pos){
        lift1.setTargetPosition(pos);
        lift2.setTargetPosition(-pos);

        if(lift1.getCurrentPosition() > pos && lift2.getCurrentPosition() < -pos){
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(1);
            lift2.setPower(1);
        }else{
            lift1.setPower(0);
            lift2.setPower(0);
        }
    }
    public void lift_pos_up(int pos){
        lift1.setTargetPosition(pos);
        lift2.setTargetPosition(-pos);

        if(lift1.getCurrentPosition() < pos && lift2.getCurrentPosition() < pos){
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(1);
            lift2.setPower(1);
        }else{
            lift1.setPower(0);
            lift2.setPower(0);
        }
    }
    public void liftDown(){
        lift_pos_down(0);
    }
    public void rrswitch(boolean s){rrmode = s;}
    public void pickup_fata(){
        liftDown();
        robot_height = Height.LOW;
//        servo_brat_jos.setPosition(1);
//        servo_brat_sus.setPosition(0.8);
    }
    public void cone_up_high_fata(){
        lift_pos_up(550);
        robot_height = Height.HIGH;
//        servo_brat_jos.setPosition(0.4);
//        servo_brat_sus.setPosition(0.2);
    }
    public void cone_up_mid_fata(){
        if(robot_height.equals(Height.HIGH)) {
            lift_pos_down(150);
            if (lift1.getCurrentPosition() < 170 && lift1.getCurrentPosition() > 130) {
                robot_height = Height.MEDIUM;
            }
        }
        else
            lift_pos_up(350);

//        servo_brat_jos.setPosition(0.4);
//        servo_brat_sus.setPosition(0.2);
    }
    public void cone_up_low_fata(){
        liftDown();
        robot_height = Height.LOW;
//        servo_brat_jos.setPosition(0.4);
//        servo_brat_sus.setPosition(0.2);
    }
//    public void cone_up_spate(){
//        servo_brat_jos.setPosition(0.4);
//        servo_brat_sus.setPosition(0.6);
//    }
}

