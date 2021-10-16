package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp (name="2020-2021 Basic Drive 247 Analog")
public class DiskSlingerCode2021 extends LinearOpMode {

    //This is where we set all of our variables so we can call them in future code
    double tgtPower = 0;
    
    // INSTATIATE SENSORS
    public GyroSensor gyroSensor;
    
    //declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //Declare Mechanism Motors
    private DcMotor IntakeMotor;
    private DcMotor WobbleGoalMotor;
    private DcMotorEx DiskFireMotor;
    private DcMotor MagWellMotor;
    
    //Declare Servos
    private CRServo WGServo;
    private Servo TriggerServo;
    private CRServo FlickerServo;
    
    //time variable
    private ElapsedTime runtime = new ElapsedTime();
    
    //motor speed variables
    double LF;
    double RF;
    double LB;
    double RB;

    //Joystick position variables
    double X1;
    double Y1;
    double X2;
    double Y2;
    
    //analog values
    double joyScale = 1.0;
    double motorMax = 1.0;
    
    @Override
    public void runOpMode() throws InterruptedException {
        //Declare and initialize limiter
        //DigitalChannel limiter = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        //limiter.setMode(DigitalChannel.Mode.INPUT);
    
        // Initialize drive motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");
    
    // DEFINE SENSOr
        gyroSensor = hardwareMap.get(GyroSensor.class, "gyroSensor");
        
        //Initialize other motors
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor"); // shooting and intake got messed up
        WobbleGoalMotor = hardwareMap.dcMotor.get("WobbleGoalMotor");
        DiskFireMotor = hardwareMap.get(DcMotorEx.class, "DiskFireMotor");
        MagWellMotor = hardwareMap.dcMotor.get("MagWellMotor");
        
        //Initialize servos
        WGServo = hardwareMap.crservo.get("WGServo");
        TriggerServo = hardwareMap.servo.get("TriggerServo");
        FlickerServo = hardwareMap.crservo.get("FlickerServo");
        
        //Initialize drive motors' direction
        //DONT CHANGE THIS CONFIRGURATION
        motorfrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorfrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorbackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorbackRight.setDirection(DcMotor.Direction.FORWARD);
    
        // May or may not be needed
        //EDIT from Michael P. -- this code is needed, all motors need to have a set direction
        //Initialize other motors' directions
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        WobbleGoalMotor.setDirection(DcMotor.Direction.REVERSE);
        DiskFireMotor.setDirection(DcMotor.Direction.FORWARD);
        MagWellMotor.setDirection(DcMotor.Direction.FORWARD); 

        //calibrate gyrosensor
        gyroSensor.calibrate();
        
        //set shooter motor to encoder-based
        DiskFireMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //Wait to start code
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        //All motors are backwards except ones that are used
        while (opModeIsActive()) {
            //all the drive code
            
            //reset speed variables
            LF = 0;
            RF = 0;
            LB = 0;
            RB = 0;
            
            //get joystick values
            Y1 = gamepad1.right_stick_y * joyScale;
            X1 = gamepad1.right_stick_x * joyScale;
            Y2 = gamepad1.left_stick_y * joyScale;
            X2 = gamepad1.left_stick_x * joyScale;
            
            //Foward/Backward
            LF += Y1;
            RF += Y1;
            LB += Y1;
            RB += Y1;
            
            //Strafing
            LF -= X2;
            RF += X2;
            LB += X2;
            RB -= X2;
            
            //Turning
            //turn right analog is imperfect
            LF -= X1;
            RF += X1;
            LB -= X1;
            RB += X1;
            
            //Wheel power limiter
            LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LB = Math.max(-motorMax, Math.min(LB, motorMax));
            RB = Math.max(-motorMax, Math.min(RB, motorMax));
            
            //set motors
            motorfrontLeft.setPower(LF);
            motorfrontRight.setPower(RF);
            motorbackLeft.setPower(LB);
            motorbackRight.setPower(RB);
            
            //intake code
            if (gamepad2.x) {
                IntakeMotor.setPower(-1);
            }
            else if (gamepad2.b) {
                IntakeMotor.setPower(1);
            }
            else {
                IntakeMotor.setPower(0);
            }
            
            //change back to gamepad2 if we want to use
             //double VARREV_POWER = 0;
            /*if (gamepad1.y) {
                VARREV_POWER = VARREV_POWER + .2;
                DiskFireMotor.setPower(VARREV_POWER);
            
            } */ //Does not work
           
            /*if (VARREV_POWER > 1)
            {
                VARREV_POWER = 0;
            }*/
            
             //mag well code
            if (gamepad2.x) { // shooting on robot
                MagWellMotor.setPower(-1);
            }
            else if (gamepad2.b) {
                MagWellMotor.setPower(1);
            }
            else {
                MagWellMotor.setPower(0);
            }
            
            //shooter code
            if (gamepad2.right_trigger > 0.2) {
                DiskFireMotor.setPower(-1);
            }
            else if (gamepad2.dpad_right) {
                DiskFireMotor.setPower(.7);
            }
            else if (gamepad2.left_trigger > 0.2) {
                DiskFireMotor.setVelocity(2000);
            }
            else {
                DiskFireMotor.setPower(0);
            }
            
            //Wobble goal motor code
            if (gamepad2.dpad_down) {
                WobbleGoalMotor.setPower(-0.5);
            }
            else if (gamepad2.dpad_up) {
                WobbleGoalMotor.setPower(0.5);
            }
            else {
                WobbleGoalMotor.setPower(0);
            }
            
            /*while (opModeIsActive()) {
                if (limiter.getState() == true) {
                    WobbleGoalMotor.setPower(0);
                }
            }*/
            //Mag well code, transports rings from intake to shooter
            //if (gamepad2.right_stick_x > 0) {
                //MagWellMotor.setPower(1);
            //}
            //else {
                //MagWellMotor.setPower(0);
            //}
            
            //wobble goal servo code
            if (gamepad1.x) {
                WGServo.setPower(1);
            }
            else if (gamepad1.b) {
                WGServo.setPower(-1);
            }
            else {
                WGServo.setPower(0);
            }
            
            //trigger arm code, loads ring into shooter
            /*if (gamepad1.b) {
                TriggerServo.setPower(1); 
            }
            else {
                TriggerServo.setPower(0); 
            }*/
            
            //trigger arm code, set arm back to load again
            
            if (gamepad2.y) {
                TriggerServo.setPosition(0); 
            }
            else if(gamepad2.a) {
                TriggerServo.setPosition(0.8);
            }
            else {
                TriggerServo.setPosition(0.4);
            }
        
        //Flicker arm code, flicks arm from intake to shooter (temporary)
            /*if (gamepad1.y) {
                FlickerServo.setPower(1);
            }
            else {
                FlickerServo.setPower(0);
            }*/
        }
    }
}
