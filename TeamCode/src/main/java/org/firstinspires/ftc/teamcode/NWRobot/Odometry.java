package org.firstinspires.ftc.teamcode.NWRobot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

public class Odometry {
    public RobotBase bot;
    public Vector2D position; // this has to be inches
    public double heading = 0;
    double startHeading = 0;

    //make any of these directions -1 to reverse the encoder without affecting the corresponding drive motor
    public int xEncoderDirection = 1;
    public int yEncoderDirection = 1;
    public int IMUDirection = -1;

    public int xEncoderPrevious = 0;
    public int yEncoderPrevious = 0;
    public double IMUPrevious = 0;
    public int xEncoderCounts=0;
    public int yEncoderCounts=0;

    public int deltaXEncoder;
    public int deltaYEncoder;
    public double deltaHeading;
    public double deltaX;
    public double deltaY;

    public double headingCorrection=0;


    final static double ENCODER_COUNTS_PER_INCH = 8192/(2.0*Math.PI*1.0);
    final static double RADIUS = 1;
    Vector2D fieldCentricDelta;
    Vector2D robotCentricDelta;

    Odometry(RobotBase robot){
        this.bot = robot;
    }

    public void initialize(LinearOpMode opMode) {

        resetAllEncoders();
        waitAllEncoders();
        setAllRunWithoutEncoders();

        position = new Vector2D(0,0);
    }
    public void setStartLocation(Vector2D startPosition, double startHeading){ //inches, and degrees
        position = new Vector2D(startPosition);
        this.startHeading = Math.toRadians(startHeading);
        heading = normalizeRadians(heading + this.startHeading);
    }

    public void resetAllEncoders(){
        bot.xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //bEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void waitAllEncoders(){
        while(bot.xEncoder.isBusy() || bot.yEncoder.isBusy()){
        }
    }
    public void setAllRunWithoutEncoders(){
        bot.xEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //lrb
    public void updateEncoders(){
        xEncoderCounts = bot.getCurrentMotorPos(bot.xEncoder);
        yEncoderCounts = bot.getCurrentMotorPos(bot.yEncoder);
        //bEncoderCounts = Agobot.getCurrentMotorPos(bEncoder);
    }

    public int getxEncoderCounts() {
        return xEncoderCounts*xEncoderDirection;
    }

    public int getyEncoderCounts() {
        return yEncoderCounts*yEncoderDirection;
    }

    public double getHeading() {
        return Math.toRadians(bot.getHeading())*IMUDirection;
    }


    public void updatePosition(){
        updateEncoders();

        deltaXEncoder =  getxEncoderCounts() - xEncoderPrevious;
        deltaYEncoder = getyEncoderCounts() - yEncoderPrevious;
        deltaHeading = getHeading() - IMUPrevious;

        xEncoderPrevious = getxEncoderCounts();
        yEncoderPrevious = getyEncoderCounts();
        IMUPrevious = getHeading();
        //bEncoderPrevious = getbEncoderCounts();

        heading = /*normalizeRadians(heading + deltaHeading);*/ normalizeRadians(IMUPrevious + startHeading+headingCorrection);


        /*if(deltaHeading == 0){ //have to do it like this because java doesn't do l'Hopital's rule
            deltaX = deltabEncoder;
            deltay = (deltalEncoder + deltarEncoder)/2;
        }else{
            double turnRadius = RADIUS*ENCODER_COUNTS_PER_INCH*(deltalEncoder + deltarEncoder)/(deltarEncoder - deltalEncoder);
            double strafeRadius = deltabEncoder/deltaHeading - BENCODER_RADIUS*ENCODER_COUNTS_PER_INCH;

            deltax = turnRadius*(Math.cos(deltaHeading) - 1) + strafeRadius*Math.sin(deltaHeading);
            deltay = turnRadius*Math.sin(deltaHeading) + strafeRadius*(1 - Math.cos(deltaHeading));
        }*/

        double straightDistance = ENCODER_COUNTS_PER_INCH*deltaXEncoder;
        double strafeDistance = ENCODER_COUNTS_PER_INCH*deltaYEncoder;

        if(deltaHeading == 0){
            deltaX = straightDistance;
            deltaY = strafeDistance;
        } else{
            deltaX = straightDistance*Math.cos(heading);
            deltaY = strafeDistance*Math.sin(heading);

        }


        //robotCentricDelta = new Vector2D(encoderToInch(deltax), encoderToInch(deltay));

        //fieldCentricDelta = new Vector2D(encoderToInch(deltay), encoderToInch(-deltax));
        fieldCentricDelta = new Vector2D(deltaY, -deltaX);
        position.add(fieldCentricDelta);
    }

    public Vector2D getRobotCentricDelta(){
        return robotCentricDelta;
    }
    public Vector2D getFieldCentricDelta(){
        return fieldCentricDelta;
    }
    public Vector2D currentPosition(){
        return position;
    }//degrees
    public double currentHeading(){
        return Math.toDegrees(heading);
    }//degrees
    public double relativeHeading(){ return Math.toDegrees(normalizeRadians(heading - startHeading)); }//degrees
    public void setHeading(double heading){ //degrees
        this.heading = normalizeRadians(Math.toRadians(heading));
    }
    public void setHeadingCorrection(double correct){
        headingCorrection = normalizeRadians(correct-this.heading+headingCorrection);
    }
    public void setRelativeHeading(double relativeHeading){
        heading = normalizeRadians(Math.toRadians(relativeHeading) + startHeading);
    }

    public float encoderToInch(double encoder) {
        return (float)(encoder/ENCODER_COUNTS_PER_INCH);
    }

    public int inchToEncoder(float inches) {
        return (int) (inches * ENCODER_COUNTS_PER_INCH);
    }

    public double normalizeRadians(double angle){
        while(angle >= 2*Math.PI) {
            angle -= 2*Math.PI;
        }
        while(angle < 0.0) {
            angle += 2*Math.PI;
        }
        return angle;
    }

}