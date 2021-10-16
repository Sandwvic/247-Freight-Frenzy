/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.robotcontroller.external.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.security.Guard;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * 
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous

public class UltimateGoalWithEncoder extends LinearOpMode{

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

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

     @TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")
     @Disabled
     public class ConceptTensorFlowObjectDetection extends LinearOpMode {
       /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
        * the following 4 detectable objects
        *  0: Ball,
        *  1: Cube,
        *  2: Duck,
        *  3: Marker (duck location tape marker)
        *
        *  Two additional model assets are available which only contain a subset of the objects:
        *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
        *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
        */
         private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
         private static final String[] LABELS = {
           "Ball",
           "Cube",
           "Duck",
           "Marker"
         };
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
           "AUj74SH/////AAABmW7EkVcRaUH6k/nXRTilrghk2aZo+nGqxj9r+nqiTE85S62vDHF2c3jlcAVifu4JyEGJ8nu661EZ5lBFR3yY3sUG8/GmDQnLmcBv8hWDTrnJa7S32G0GboQxfGjK4Hfg9JPnE7gPeoRLakND+8d/i4UAmJ2bxXQowKa6tqUsMKkl097PNN9qyWh+sQjV3gJIDg5LFeLXbbWXTQb+bNixxcXm5xF0ctjOZhXp3kKR1qVeOidqM3qH6QyAIGlJV7e4vT/Ph7XmQ1TbvdjckVCWy3gTegef5/H1h36ECTd0zkvF/McF7g+Wb5MJVPpX2jrqHRezPtoKBgKPDnEftXG2cvvZaZ0IxJmyYayRvzdRFY1F";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

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
    
    //Declare servoes
    private Servo TriggerServo;
    private Servo GuardServo1;
    private Servo GuardServo2;
    
    //the time Object
    private ElapsedTime runtime=new ElapsedTime();
    
    //integer to stop code with 0 rings from constantly running
    private int runOnce;
            
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
            tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        knockGuards();
        if (opModeIsActive()) {
            //Initialize drive motors
            motorfrontLeft = hardwareMap.dcMotor.get("motorfrontLeft");
            motorfrontRight = hardwareMap.dcMotor.get("motorfrontRight");
            motorbackLeft = hardwareMap.dcMotor.get("motorbackLeft");
            motorbackRight = hardwareMap.dcMotor.get("motorbackRight");
    
            //Initialize other motors
            IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
            WobbleGoalMotor = hardwareMap.dcMotor.get("WobbleGoalMotor");
            DiskFireMotor = hardwareMap.get(DcMotorEx.class,"DiskFireMotor");
            MagWellMotor = hardwareMap.dcMotor.get("MagWellMotor");
            
            //initialize servos
            TriggerServo = hardwareMap.servo.get("TriggerServo");
            GuardServo1 = hardwareMap.servo.get("GuardServo1");
            GuardServo2 = hardwareMap.servo.get("GuardServo2");
            //Initialize drive motors' direction
            //DONT CHANGE THIS CONFIRGURATION
            motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
            motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
            motorbackLeft.setDirection(DcMotor.Direction.FORWARD);
            motorbackRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize encoders
        initEncoder();
            
            runOnce = 1;
           
            /* Move using Encoders */
            
            //start backwards
            //Code for detecting rings
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        //telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if((updatedRecognitions.size() == 0) && (runOnce == 1)) {
                          //empty list. no objects recognized.
                          telemetry.addData("TFOD", "No items detected.");
                          telemetry.addData("Target Zone", "A");
                          //robot code
                          encoderDrive(DRIVE_SPEED, 122, 122/*, 6*/);
                          //encoderDrive(TURN_SPEED, -2, 2, 1);
                          
                          loadRing();
                          ShootAll();
                          Stop(0.2);
                          //encoderDrive(DRIVE_SPEED, 5, 5, 9);//This needs to be adjusted, it currently moves the robot too much
                          encoderDrive(DRIVE_SPEED, 25, 25/*, 4*/);
                          encoderDrive(TURN_SPEED, 42, -42/*, 5*/);
                          encoderDrive(DRIVE_SPEED, -18, -18/*, 4*/);
                          setsDownWG(0.6);
                          Stop(0.2);
                          encoderDrive(DRIVE_SPEED, 36, 36/*, 10*/);
                         
                          
                          runOnce--;
                        } else {
                            // list is not empty.
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());
                            // check label to see which target zone to go after.
                            if ((recognition.getLabel().equals("Single")) && (runOnce == 1)) {
                                telemetry.addData("Target Zone", "B");
                                //robot code
                                encoderDrive(DRIVE_SPEED, 122, 122/*, 108*/);
                                loadRing();
                                ShootAll();
                                Stop(0.2);
                                encoderDrive(TURN_SPEED, -90, 90/*, 9*/);
                                encoderDrive(DRIVE_SPEED, -48, -48/*, 9*/);
                                setsDownWG(0.8);
                                Stop(0.2);
                                encoderDrive(DRIVE_SPEED, 22, 22/*, 9*/);
                                sleep(100);
                                
                                runOnce--;
                            } else if ((recognition.getLabel().equals("Quad")) && (runOnce == 1)) {
                                telemetry.addData("Target Zone", "C");
                                //robot code
                                encoderDrive(DRIVE_SPEED, 122, 122/*, 10*/);
                                loadRing();
                                ShootAll();
                                Stop(0.1);
                                encoderDrive(DRIVE_SPEED, 95, 95/*, 9*/);
                                encoderDrive(TURN_SPEED, 58, -58/*, 9*/);
                                encoderDrive(DRIVE_SPEED, -28, -28/*, 9*/);
                                setsDownWG(0.8);
                                Stop(0.1);
                                encoderDrive(DRIVE_SPEED, 26, 26/*, 9*/);
                                encoderDrive(TURN_SPEED, 28, -28/*, 9*/);
                                encoderDrive(DRIVE_SPEED, 78, 78/*, 10*/);
                                runOnce--;
                            } else {
                                telemetry.addData("Target Zone", "Unknown");
                            }
                        }
                    }
                    telemetry.update();
                  } 
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }
    
    //FUNCTIONS MADE HERE
    //knock downs guard
    public void knockGuards()
    {
        GuardServo1.setPosition(-1);
        GuardServo2.setPosition(1);
    }
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
    
    //takes rings into robot
    public void intake(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            IntakeMotor.setPower(-1);
        }
    }
        
    //Shoots rings
    public void shoot(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            DiskFireMotor.setPower(1);
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
    
    //stops intake
    public void stopIntake(double time){
        double run=(runtime.time()+time);
        while(runtime.time()<run){
            IntakeMotor.setPower(0);
        }
    }
    
    //loads ring into shooter
    public void loadRing(){
        TriggerServo.setPosition(1);
    }
    //wait function
    //public void waitThen(double time, )
    
    //code for intaking and shooting 1 ring
    public void ShootAll() {
        for(int x = 0; x < 2; x++)
        {
            PIDShoot(2000);//encoderShoot(50)
            stopIntake(1.0);
            intake(0.5);
            magWell(0.5);
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
       }
   }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches
                             /*double timeoutS*/) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = motorfrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = motorfrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newbackLeftTarget = motorbackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbackRightTarget = motorbackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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
    
    public void encoderStrafe(double speed,
                             double frontInches, double backInches
                             /*double timeoutS*/) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = motorfrontLeft.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);
            newfrontRightTarget = motorfrontRight.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);
            newbackLeftTarget = motorbackLeft.getCurrentPosition() + (int)(backInches * COUNTS_PER_INCH);
            newbackRightTarget = motorbackRight.getCurrentPosition() + (int)(backInches * COUNTS_PER_INCH);
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
    
    //shoot with PID controller
    
    public void PIDShoot(double ticks) {
        DiskFireMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DiskFireMotor.setVelocity(ticks);
        
        //telemetry for displaying velocity values
        while (opModeIsActive() &&
                   (DiskFireMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                DiskFireMotor.getVelocity());
                telemetry.addLine("Velocity: " + DiskFireMotor.getVelocity());

                telemetry.update();
        }
    }
    
    /*  For shooter
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    
    public void encoderShoot(double speed,
                             double Inches,
                             double timeoutS) {
        int newShootingValueTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newShootingValueTarget = DiskFireMotor.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            DiskFireMotor.setTargetPosition(newShootingValueTarget);

            // Turn On RUN_TO_POSITION
            DiskFireMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DiskFireMotor.setPower(Math.abs(speed)); // problems may occur if shooter is set to less than 1, changes may be necessary

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (DiskFireMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                newShootingValueTarget);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            DiskFireMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            DiskFireMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            DiskFireMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // delete if code not meant to pause
        }
    }
    
    /*  For shooter
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    
    public void encoderIntake(double speed,
                             double Inches,
                             double timeoutS) {
        int newIntakeValueTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newIntakeValueTarget = IntakeMotor.getCurrentPosition() + (int)(Inches * INTAKE_COUNTS_PER_INCH);
            IntakeMotor.setTargetPosition(newIntakeValueTarget);

            // Turn On RUN_TO_POSITION
            IntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            IntakeMotor.setPower(Math.abs(speed)); // problems may occur if intake is set to less than 1, changes may be necessary

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (IntakeMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                newIntakeValueTarget);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            IntakeMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            IntakeMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // delete if code not meant to pause
        }
    }

    /* Step through each leg of the path,
     * Note: Reverse movement is obtained by setting a negative distance (not speed)
     */
    public void encoderMove() {
        // sample code
        // encoderDrive(TURN_SPEED,   -5, 5, 4.0);  // S2: Turn Left 5 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 60, 60/*, 4.0*/);  // S1: Forward 56 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   -12, 12, 4.0);  // S2: Turn Left 12 Inches with 4 Sec timeout
        // encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
    } 
 
    /* Initialize encoders */
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

}


