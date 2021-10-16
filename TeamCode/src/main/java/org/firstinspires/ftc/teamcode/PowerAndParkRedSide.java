package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.security.Guard;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
public class PowerAndParkRedSide extends LinearOpMode {
    
    /* Encoder Specific defines */
    private static final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION    = 0.3 ;     // This is < 1.0 if geared UP //this is for wheels only, make new variables for other motors
    private static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference //this is for wheels only, make new variables for other motors
    private static final double INTAKE_DIAMETER_INCHES  = 3.0 ;     // For figuring circumference of intake
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                              (WHEEL_DIAMETER_INCHES * 3.1415); //this is for wheels only, make new variables for other motors
    //private static final double COUNTS_PER_INCH_CAL     = COUNTS_PER_INCH + 300;
    private static final double INTAKE_COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                              (INTAKE_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED             = 0.4;//1.0; //this is for wheels only, make new variables for other motors
    private static final double MAX_SPEED               = 1.0;
    private static final double TURN_SPEED              = 0.4; //this is for wheels only, make new variables for other motors

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
    private Servo GuardServo1;
    private Servo GuardServo2;
    
    //the time Object
    private ElapsedTime runtime=new ElapsedTime();
    
    //FUNCTIONS MADE HERE
    //knock down guards
    public void knockGuards()
    {
        GuardServo1.setPosition(-1);
        GuardServo2.setPosition(1);
    }
    //move backwards
    public void backwards(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-0.5);
            motorfrontRight.setPower(-0.5);
            motorbackLeft.setPower(-0.5);
            motorbackRight.setPower(-0.5);
        }
    }
    
    //moves forward
    public void forward(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(0.5);
            motorfrontRight.setPower(0.5);
            motorbackLeft.setPower(0.5);
            motorbackRight.setPower(0.5);
        }
    }
    
    //turn left
    public void turnL(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-0.5);
            motorfrontRight.setPower(0.5);
            motorbackLeft.setPower(-0.5);
            motorbackRight.setPower(0.5);
        }
    }
    //turn right
    public void turnR(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(0.5);
            motorfrontRight.setPower(-0.5);
            motorbackLeft.setPower(0.5);
            motorbackRight.setPower(-0.5);
        }
    }
    
    //strafe right
    public void strafeR(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(0.5);
            motorfrontRight.setPower(-0.5);
            motorbackLeft.setPower(-0.5);
            motorbackRight.setPower(0.5);
        }
    }
    
    //strafe left
    public void strafeL(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            motorfrontLeft.setPower(-0.5);
            motorfrontRight.setPower(0.5);
            motorbackLeft.setPower(0.5);
            motorbackRight.setPower(-0.5);
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
            DiskFireMotor.setPower(0.7);
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
            MagWellMotor.setPower(1);
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
            TriggerServo.setPosition(1);
        }
    }
    
    //stops intake
    public void stopIntake(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            IntakeMotor.setPower(0);
        }
    }
    
    //Stops all motors except shooter
    public void StopButShoot(double time){
       double run=(runtime.time()+time);
       while(runtime.time()<run){
           motorfrontLeft.setPower(0);
           motorfrontRight.setPower(0);
           motorbackLeft.setPower(0);
           motorbackRight.setPower(0);
           WobbleGoalMotor.setPower(0);
           IntakeMotor.setPower(0);
           MagWellMotor.setPower(0);
           WGServo.setPower(0);
           //TriggerServo.setPower(0);
           FlickerServo.setPower(0);
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
           //TriggerServo.setPower(0);
           FlickerServo.setPower(0);
       }
   }
   
   public void encoderDrive(double speed,
                             double FLBRInches, double BLFRInches
                             /*double timeoutS*/) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = motorfrontLeft.getCurrentPosition() + (int)(FLBRInches * COUNTS_PER_INCH);
            newfrontRightTarget = motorfrontRight.getCurrentPosition() + (int)(BLFRInches * COUNTS_PER_INCH);
            newbackLeftTarget = motorbackLeft.getCurrentPosition() + (int)(BLFRInches * COUNTS_PER_INCH);
            newbackRightTarget = motorbackRight.getCurrentPosition() + (int)(FLBRInches * COUNTS_PER_INCH);
            motorfrontLeft.setTargetPosition(newfrontLeftTarget);
            motorfrontRight.setTargetPosition(newfrontRightTarget);
            motorbackLeft.setTargetPosition(newbackLeftTarget);
            motorbackRight.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            motorfrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorfrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorbackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorbackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            sleep(2000);
            motorfrontLeft.setPower(Math.abs(speed));
            motorbackLeft.setPower(Math.abs(speed));
            motorfrontRight.setPower(Math.abs(speed));
            motorbackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   //(runtime.seconds() < timeoutS) &&
                   (motorfrontLeft.isBusy() && motorfrontRight.isBusy() &&
                   motorbackLeft.isBusy() && motorbackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                newfrontLeftTarget,  newfrontRightTarget,
                newbackLeftTarget, newbackRightTarget);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            motorfrontLeft.getCurrentPosition(),
                                            motorfrontRight.getCurrentPosition(),
                                            motorbackLeft.getCurrentPosition(),
                                            motorbackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorfrontLeft.setPower(0);
            motorfrontRight.setPower(0);
            motorbackLeft.setPower(0);
            motorbackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorfrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorfrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorbackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorbackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // delete if code not meant to pause
        }
    }
    
    public void PIDShoot(double ticks) {
        DiskFireMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DiskFireMotor.setVelocity(ticks);
        
        //telemetry for displaying velocity values
        while (opModeIsActive() &&
                   (DiskFireMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                DiskFireMotor.getVelocity());

                telemetry.update();
        }
    }
    
    public void initEncoder() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motorfrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DiskFireMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        motorfrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DiskFireMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                motorfrontLeft.getCurrentPosition(),
                motorfrontRight.getCurrentPosition(), 
                motorbackLeft.getCurrentPosition(),
                motorbackRight.getCurrentPosition(),
                DiskFireMotor.getCurrentPosition(),
                IntakeMotor.getCurrentPosition());
        telemetry.update();
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
        DiskFireMotor = hardwareMap.get(DcMotorEx.class, "DiskFireMotor");
        MagWellMotor = hardwareMap.dcMotor.get("MagWellMotor");
        
        //Initialize servos
        WGServo = hardwareMap.crservo.get("WGServo");
        TriggerServo = hardwareMap.servo.get("TriggerServo");
        GuardServo1 = hardwareMap.servo.get("GuardServo1");
        GuardServo2 = hardwareMap.servo.get("GuardServo2");
        FlickerServo = hardwareMap.crservo.get("FlickerServo");
        
        //Initialize drive motors' direction
        //DONT CHANGE THIS CONFIRGURATION
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorbackRight.setDirection(DcMotor.Direction.REVERSE);
        //call functions here
        waitForStart();
        knockGuards();
        encoderDrive(DRIVE_SPEED, 120, 120);
        Stop(1);
        loadRing(1);
        Stop(0.2);
        //strafeR(1.0);
        Stop(0.2);
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
        PIDShoot(1700);
        sleep(2000);
        magWell(0.65);
        Stop(1);
        encoderDrive(DRIVE_SPEED, -25, 25);
        Stop(1);
        PIDShoot(1700);
        sleep(2000);
        magWell(0.37);
        Stop(1);
        encoderDrive(DRIVE_SPEED, -22, 22);
        Stop(1);
        PIDShoot(1700);
        sleep(2000);
        magWell(1);
        Stop(1);
        encoderDrive(DRIVE_SPEED, 20, 20);
        Stop(1);
        /*shoot(4);
        intake(0.65);
        magWell(0.65);
        StopButShoot(0.2);
        turnR(0.07);
        StopButShoot(0.2);
        intake(0.65);
        StopButShoot(0.2);
        turnR(0.03);
        StopButShoot(0.2);
        intake(3.0);
        Stop(0.2);
        forward(0.4);*/
        /*strafeR(0.5);
        strafeL(0.5);
        strafeR(0.5);
        strafeL(0.5);
        strafeR(0.5);
        strafeL(0.5);*/
   }
}
