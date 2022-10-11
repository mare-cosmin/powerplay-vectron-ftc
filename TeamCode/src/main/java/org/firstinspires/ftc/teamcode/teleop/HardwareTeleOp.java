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
    public DcMotor motor_brat = null;
    public DcMotor motor_brat2 = null;

    public boolean no_roti_test = true;
    private boolean no_brat_test = false;

    private boolean only_roti_test = false;


    //define servos
    public Servo gripper = null;


    public void initialize(HardwareMap hardwareMap){
        //initialize motors
        if(!no_roti_test) {
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            motor_brat = hardwareMap.get(DcMotor.class, "motor_brat");
            motor_brat2 = hardwareMap.get(DcMotor.class, "motor_brat2");

            //initialize servos

            //set motor directions
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            motor_brat.setDirection(DcMotor.Direction.FORWARD);
            motor_brat2.setDirection(DcMotor.Direction.REVERSE);

            //set motor modes
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //set motor powers
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            motor_brat.setPower(0);
            motor_brat2.setPower(0);
        }
        if(!only_roti_test) {
            gripper = hardwareMap.get(Servo.class, "gripper");
            //set servo directions
            gripper.setDirection(Servo.Direction.FORWARD);
            //set servo positions
            gripper.setPosition(0);
        }

        motor_brat = hardwareMap.get(DcMotor.class, "motor_brat");
        motor_brat2 = hardwareMap.get(DcMotor.class, "motor_brat2");
        motor_brat.setDirection(DcMotor.Direction.FORWARD);
        motor_brat2.setDirection(DcMotor.Direction.REVERSE);
        motor_brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_brat.setPower(0);
        motor_brat2.setPower(0);


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
    public void move_brat(double power){
        motor_brat.setPower(power);
        motor_brat2.setPower(power);
    }

    public void move(double power,double distance, int direction) {
        //moves with respect to direction:
        //if direction=1, moves forward
        //if direction=2, moves backward
        //if direction=3, moves left
        //if direction=4, moves right

        stopAndResetEncodersCHASSIS();

        /*
        int target =  (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_FORWARD);
        if (direction == 1){
            leftFront.setTargetPosition(target);
            rightFront.setTargetPosition(target);
            leftBack.setTargetPosition(target);
            rightBack.setTargetPosition(target);
        }else if (direction == 2) {
            leftFront.setTargetPosition(-target);
            rightFront.setTargetPosition(-target);
            leftBack.setTargetPosition(-target);
            rightBack.setTargetPosition(-target);
        }else if (direction == 3) {
            leftFront.setTargetPosition(-target);
            rightFront.setTargetPosition(target);
            leftBack.setTargetPosition(target);
            rightBack.setTargetPosition(-target);
        }else if (direction == 4) {
            leftFront.setTargetPosition(target);
            rightFront.setTargetPosition(-target);
            leftBack.setTargetPosition(-target);
            rightBack.setTargetPosition(target);
        }else{
            telmetry.addData("Error","Direction not valid");
        }
        */


        runToPositionCHASSIS();

        runtime.reset();

        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();
    }
    public void turn(double power, double distance, int direction) {
        //turns with respect to direction:
        //if direction=1, turns left
        //if direction=2, turns right

        stopAndResetEncodersCHASSIS();

        /*
        int target =  (int)(distance * COUNTS_CM_CHASSIS * CORECTIE_TURN);
        if (direction == 1){
            leftFront.setTargetPosition(-target);
            rightFront.setTargetPosition(target);
            leftBack.setTargetPosition(-target);
            rightBack.setTargetPosition(target);
        }else if (direction == 2) {
            leftFront.setTargetPosition(target);
            rightFront.setTargetPosition(-target);
            leftBack.setTargetPosition(target);
            rightBack.setTargetPosition(-target);
        }else{
            telemetry.addData("Error", "Invalid direction");
            telemetry.update();
        }
        */
        runToPositionCHASSIS();

        runtime.reset();

        setPowerCHASSIS(power);

        setPowerZeroCHASSIS();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAndResetEncodersCHASSIS();
    }

    //functii pentru celelalte motoare

    public void runToPositionBRAT(){
        motor_brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setPowerBRAT(double power){
        motor_brat.setPower(power);
        motor_brat2.setPower(power);
    }
    public void setPowerZeroBRAT(){
        motor_brat.setPower(0);
        motor_brat2.setPower(0);
    }
    public void stopAndResetEncodersBRAT(){
        motor_brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_brat2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void closeGripper(){
        gripper.setPosition(1);
    }
    public void openGripper(){
        gripper.setPosition(0);
    }
}

