package org.firstinspires.ftc.teamcode.teleop;

//import necessary ftc hardware packages
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareTeleOp {
    //define motors
    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public DcMotor lift_left = null;
    public DcMotor lift_right = null;

    public boolean rrmode = true;

    public boolean chassis_test = true;
    public boolean lift_test = true;
    public boolean gripper_test = true;
    public boolean servo_brat_test = true;
    public boolean all_in = false;

    public double angle = 0;
    public double distance = 0;

    double COUNTS_CM_CHASSIS = 537.7 / (9.6 * 3.1415);

    double CORECTIE_FORWARD = 0.949710;
    double CORECTIE_REVERSE = 0.927435;
    double CORECTIE_MOVELEFT = 1.075473;
    double CORECTIE_MOVERIGHT = 1.095052;
    double CORECTIE_ROTATERIGHT = 0.5;
    double CORECTIE_ROTATELEFT = 0.5;
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
            leftRear = hardwareMap.get(DcMotor.class, "leftRear");
            rightRear = hardwareMap.get(DcMotor.class, "rightRear");
            //set motor directions
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.FORWARD);

            //set motor modes
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //set motor powers
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }
        if(gripper_test || all_in) {
            //initialize servos
            gripper = hardwareMap.get(Servo.class, "gripper");
            gripper.setDirection(Servo.Direction.FORWARD);
            gripper.setPosition(0.6);
        }
        if(servo_brat_test || all_in) {
            servo_brat_jos = hardwareMap.get(Servo.class, "servo_brat_jos");
            servo_brat_sus = hardwareMap.get(Servo.class, "servo_brat_sus");

            servo_brat_jos.setDirection(Servo.Direction.FORWARD);
            servo_brat_sus.setDirection(Servo.Direction.FORWARD);
            servo_brat_jos.setPosition(1);
            servo_brat_sus.setPosition(0.7);
        }

        if(lift_test || all_in) {
            lift_left = hardwareMap.get(DcMotor.class, "lift_left");
            lift_left.setDirection(DcMotor.Direction.FORWARD);
            lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_left.setPower(0);

            lift_right = hardwareMap.get(DcMotor.class, "lift_right");
            lift_right.setDirection(DcMotor.Direction.FORWARD);
            lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_right.setPower(0);

            lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public void runToPositionCHASSIS(){
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPowerCHASSIS(double power){

        rightFront.setPower(power);
        rightRear.setPower(power);
        leftRear.setPower(power);
        leftFront.setPower(power);
    }
    public void setPowerZeroCHASSIS(){

        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        leftFront.setPower(0);
    }
    public void stopAndResetEncodersCHASSIS(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //functii pentru celelalte motoare

    public void closeGripper(){
        gripper.setPosition(1);
    }
    public void openGripper(){
        gripper.setPosition(0.6);
    }
    public void lift_pos_down(int pos){
        lift_left.setTargetPosition(pos);
        lift_right.setTargetPosition(pos);

        if(lift_left.getCurrentPosition()>=pos+10){
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_left.setPower(0.5);
            lift_right.setPower(0.5);
        }
        else{
            lift_left.setPower(0);
            lift_right.setPower(0);
        }
    }
    public void lift_pos_up(int pos){
        lift_left.setTargetPosition(pos);
        lift_right.setTargetPosition(pos);
        if(lift_left.getCurrentPosition() <= pos-10){
            lift_left.setPower(1);
            lift_right.setPower(1);
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }else{
            lift_left.setPower(0);
            lift_right.setPower(0);
        }
    }
    public void moveForward(double power,double distance) {

        stopAndResetEncodersCHASSIS();

        int target =  (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_FORWARD);

        rightFront.setTargetPosition(-5000);
        leftFront.setTargetPosition(-5000);
        leftRear.setTargetPosition(-5000);
        rightRear.setTargetPosition(-5000);

        runToPositionCHASSIS();

        runtime.reset();

        if(rightRear.getCurrentPosition() >= rightRear.getTargetPosition())
            setPowerCHASSIS(power);
        else
            setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();
    }

    public void moveReverse(double power, double distance){

        stopAndResetEncodersCHASSIS();

        int target = (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_REVERSE);

        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(target);
        leftRear.setTargetPosition(target);
        rightRear.setTargetPosition(target);

        runToPositionCHASSIS();

        runtime.reset();

        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();


    }

    public void moveLeft(double power, double distance){

        stopAndResetEncodersCHASSIS();

        int target = (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_MOVELEFT);

        rightFront.setTargetPosition(-target);
        leftFront.setTargetPosition(target);
        leftRear.setTargetPosition(-target);
        rightRear.setTargetPosition(target);

        runToPositionCHASSIS();

        runtime.reset();

        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();


    }

    public void moveRight(double power, double distance){

        stopAndResetEncodersCHASSIS();

        int target = (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_MOVERIGHT);

        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(-target);
        leftRear.setTargetPosition(target);
        rightRear.setTargetPosition(-target);

        runToPositionCHASSIS();

        runtime.reset();

        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();

    }

    public void rotateRight(double power, double distance){

        stopAndResetEncodersCHASSIS();

        int target = (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_ROTATERIGHT);

        rightFront.setTargetPosition(target);
        leftFront.setTargetPosition(-target);
        leftRear.setTargetPosition(-target);
        rightRear.setTargetPosition(target);

        runToPositionCHASSIS();

        runtime.reset();

        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();

    }

    public void rotateLeft(double power, double distance){

        stopAndResetEncodersCHASSIS();

        int target = (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_ROTATELEFT);

        rightFront.setTargetPosition(-target);
        leftFront.setTargetPosition(target);
        leftRear.setTargetPosition(target);
        rightRear.setTargetPosition(-target);

        runToPositionCHASSIS();

        runtime.reset();

        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();

    }
    public void liftDown(){
        lift_pos_down(0);
    }
    public void rrswitch(boolean s){rrmode = s;}
    public void pickup_fata(){
        liftDown();
        robot_height = Height.LOW;
        servo_brat_jos.setPosition(0.52);
        servo_brat_sus.setPosition(0.6);
    }
    public void cone_up_high_fata(){
        lift_pos_up(650);
        robot_height = Height.HIGH;
        servo_brat_jos.setPosition(1);
        servo_brat_sus.setPosition(1);
    }
    public void cone_up_mid_fata(){
        if(robot_height.equals(Height.LOW)){
            lift_pos_up(500);
            robot_height = Height.MEDIUM;
        }else if(robot_height.equals(Height.HIGH)){
            lift_pos_down(500);
            robot_height = Height.MEDIUM;
        }
        servo_brat_jos.setPosition(1);
        servo_brat_sus.setPosition(1);
    }
    public void cone_up_low_fata(){
        liftDown();
        if(!lift_left.isBusy()){
            robot_height = Height.LOW;
            servo_brat_jos.setPosition(1);
            servo_brat_sus.setPosition(1);
        }
    }
//    public void cone_up_spate(){
//        servo_brat_jos.setPosition(0.4);
//        servo_brat_sus.setPosition(0.6);
//    }
}

