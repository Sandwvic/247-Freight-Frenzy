package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class ShootAndPark extends LinearOpMode {

    //declare Drive Motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;

    //Declare Mechanism Motors
    private DcMotor IntakeMotor;
    private DcMotor WobbleGoalMotor;
    private DcMotor DiskFireMotor;
    private DcMotor MagWellMotor;
    
    //Declare Servos
    private CRServo WGServo;
    private Servo TriggerServo;
    private CRServo FlickerServo; 
    private Servo GuardServo1;
    private Servo GuardServo2;
    //the time Object
    private ElapsedTime runtime=new ElapsedTime();
    
    //FUNCTIONS MADE HERE
    //move backwards
    public void backwards(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(-1);
        }
    }
    
    //moves forward
    public void forward(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(1);
        }
    }
    
    //turn left
    public void turnL(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(1);
        }
    }
    //turn right
    public void turnR(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(-1);
        }
    }
    
    //strafe right
    public void strafeR(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-1);
            motorfrontRight.setPower(1);
            motorbackLeft.setPower(1);
            motorbackRight.setPower(-1);
        }
    }
    
    //strafe left
    public void strafeL(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(1);
            motorfrontRight.setPower(-1);
            motorbackLeft.setPower(-1);
            motorbackRight.setPower(1);
        }
    }
    
    //flicks ring from intake to servo
    public void flick(double time){
       double run=(runtime.time()+time);
       while(runtime.time()<run){
           FlickerServo.setPower(1);
       }
    }
    
    //takes rings into robot
    public void intake(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            IntakeMotor.setPower(1);
        }
    }
        
    //Shoots rings
    public void shoot(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            DiskFireMotor.setPower(1);
        }
    }
    
    //Shoots rings with half speed
    public void shootSlow(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            DiskFireMotor.setPower(0.5);
        }
    }
    
    //Picks up or sets down wobble goal
    //If sets down change name
    public void pickUpWG(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            WobbleGoalMotor.setPower(-1);
        }
    }
    
    //picks up or sets down wobble goal
    //If picks up change name
    public void setsDownWG(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            WobbleGoalMotor.setPower(1);
        }
    }
    
    //moves rings up ramp
    public void magWell(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            MagWellMotor.setPower(-1);
        }
    }
    
    //holds or lets go of wobble goal
    //if lets go change name
    public void holdWG(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            WGServo.setPower(1);
        }
    }
    
    //Holds or lets go of wobble goal
    //if holds change name
    public void letGoOfWG(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            WGServo.setPower(-1);
        }
    }
    
    //loads ring into shooter
    public void loadRing(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            TriggerServo.setPosition(0.2);
        }
    }
    
    //stops intake
    public void stopIntake(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            IntakeMotor.setPower(0);
        }
    }
    
    //Stops all motors from moving
    public void Stop(double time){
       double run=(runtime.time()+time);
       while(runtime.time()<run){
           motorfrontLeft.setPower(0);
           motorfrontRight.setPower(0);
           motorbackLeft.setPower(0);
           motorbackRight.setPower(0);
           DiskFireMotor.setPower(0);
           WobbleGoalMotor.setPower(0);
           IntakeMotor.setPower(0);
           MagWellMotor.setPower(0);
           WGServo.setPower(0);
           TriggerServo.setPosition(0);
           FlickerServo.setPower(0);
       }
   }
   //Code starts here
   @Override
   public void runOpMode() throws InterruptedException {
        //Initialize drive motors
        motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
        motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
        motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
        motorbackRight = hardwareMap.dcMotor.get("motorbackRight");
    
        //Initialize other motors
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        WobbleGoalMotor = hardwareMap.dcMotor.get("WobbleGoalMotor");
        DiskFireMotor = hardwareMap.dcMotor.get("DiskFireMotor");
        MagWellMotor = hardwareMap.dcMotor.get("MagWellMotor");
        
        //Initialize servos
        WGServo = hardwareMap.crservo.get("WGServo");
        TriggerServo = hardwareMap.servo.get("TriggerServo");
        FlickerServo = hardwareMap.crservo.get("FlickerServo");
        
        //Initialize drive motors' direction
        //DONT CHANGE THIS CONFIRGURATION
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorbackRight.setDirection(DcMotor.Direction.REVERSE);
        //call functions here
        waitForStart();
        forward(1.6);
        Stop(1);
        loadRing(1);
        /* shoot(8);
        for (int count; count < 3; count++)
        {
            intake(0.65);
            stopIntake(1);
        } //this is just the test (maybe used to save time if necessary)
        for (int count; count < 3; count++)
        {
            shoot(4);
            intake(0.65);
            Stop(1);
        } */
        /*shoot(4);
        intake(0.65);
        magWell(0.65);
        Stop(1);
        shoot(4.2);
        intake(0.65);
        magWell(0.65);
        Stop(1);
        shoot(4);
        intake(0.65);
        magWell(0.65);
        shoot(1);
        Stop(1);*/
        shoot(6);
        intake(0.65);
        magWell(0.65);
        stopIntake(1);
        intake(0.65);
        stopIntake(1);
        intake(0.65);
        Stop(1);
        forward(0.4);
        Stop(1);
   }
}
