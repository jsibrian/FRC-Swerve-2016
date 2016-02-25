
package org.usfirst.frc.team1661.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {
	   int width;
	   int length;
	   double R;
	   double LRscaleFactor;
	   double WRscaleFactor;
	   double RADIANS;
	   private Joystick XC=new Joystick(0);
   		private CANTalon fld; 
		private CANTalon frd; 
		private CANTalon bld; 
		private CANTalon brd; 
		private CANTalon fls; 
		private CANTalon frs; 
		private CANTalon bls; 
		private CANTalon brs;
    public Robot() {
    	width = 28;
    	length = 28;
    	R = Math.sqrt((width * width) + (length * length));
    	LRscaleFactor = length / R;
    	WRscaleFactor = width / R;
    	RADIANS = 180 / Math.PI;
    	fld = new CANTalon(1);
        fls = new CANTalon(2);
        frd = new CANTalon(3);
        frs = new CANTalon(4);
        bld = new CANTalon(5);
        bls = new CANTalon(6);
        brd = new CANTalon(7);
        brs = new CANTalon(8);
        fls.changeControlMode(CANTalon.TalonControlMode.Position);
        frs.changeControlMode(CANTalon.TalonControlMode.Position);
        bls.changeControlMode(CANTalon.TalonControlMode.Position);
        brs.changeControlMode(CANTalon.TalonControlMode.Position);
        fls.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        frs.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        bls.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        brs.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        fls.configEncoderCodesPerRev(7);
        frs.configEncoderCodesPerRev(7);
        bls.configEncoderCodesPerRev(7);
        brs.configEncoderCodesPerRev(7);
        fls.setPID(1, 0, 0);
        frs.setPID(1, 0, 0);
        bls.setPID(1, 0, 0);
        brs.setPID(1, 0, 0);
    }
    
    public void robotInit() {
            }

	
    public void autonomous() {
    	
    }

    public void operatorControl() {
    	double frontLeftAngle=0,frontRightAngle=0,rearLeftAngle=0,rearRightAngle=0;
    	fls.setPosition(0);
	    frs.setPosition(0);
	    brs.setPosition(0);
	    bls.setPosition(0);
	    double A,B,C,D;
	    double panglefl=0;
	    double panglefr=0;
	    double panglebl=0;
	    double panglebr=0;
    	while (isOperatorControl() && isEnabled()) {
        	double strafe = XC.getRawAxis(0);
            double fwd =-XC.getRawAxis(1);
            double rotation = XC.getRawAxis(4);
            double fldSpeed=0;
            double frdSpeed=0;
            double bldSpeed=0;
            double brdSpeed=0;
            if(XC.getRawButton(1)){
                fls.set(0);
                frs.set(0);
                bls.set(0);
                brs.set(0);
            }
            if(Math.abs(fwd)>.2||Math.abs(strafe)>.2||Math.abs(rotation)>.2){
            if(Math.abs(XC.getRawAxis(4))>.2){
            A = strafe - rotation * LRscaleFactor;
            B = strafe + rotation * LRscaleFactor;
            C = fwd - rotation * WRscaleFactor;
            D = fwd + rotation * WRscaleFactor;
            }
            else{
                A = strafe;
                B = strafe;
                C = fwd;
                D = fwd;
            }
            fldSpeed = Math.sqrt((B * B) + (D * D));
            frdSpeed = Math.sqrt((B * B) + (C * C));
            bldSpeed = Math.sqrt((A * A) + (D * D));
            brdSpeed = Math.sqrt((A * A) + (C * C));
            double maxSpeed = Math.max(fldSpeed, frdSpeed);
            maxSpeed = Math.max(maxSpeed, bldSpeed);
            maxSpeed = Math.max(maxSpeed, brdSpeed);
            if (maxSpeed > 1)
            {
               fldSpeed = fldSpeed / maxSpeed;
               frdSpeed = frdSpeed / maxSpeed;
               bldSpeed = bldSpeed / maxSpeed;
               brdSpeed = brdSpeed / maxSpeed;
            }
            if (D == 0 && B == 0)
            {
               frontLeftAngle = 0;
            }
            else
            {
               frontLeftAngle = (Math.atan2(D, B) * RADIANS);
          
               frontLeftAngle = (-frontLeftAngle + 360 +90) % 360;
            }
            if (C == 0 && B == 0)
            {
               frontRightAngle = 0;
            }
            else
            {
               frontRightAngle = (Math.atan2(C, B) * RADIANS);
               
               frontRightAngle = (-frontRightAngle + 360+90) % 360;
            }
            if (D == 0 && A == 0)
            {
               rearLeftAngle = 0;
            }
            else
            {
               rearLeftAngle = (Math.atan2(D, A) * RADIANS);
              
               rearLeftAngle = (-rearLeftAngle + 360+90) % 360;
            }
            if (C == 0 && A == 0)
            {
               rearRightAngle = 0;
            }
            else
            {
               rearRightAngle = (Math.atan2(C, A) * RADIANS);
             
               rearRightAngle = (-rearRightAngle + 360+90) % 360;
            }
            SmartDashboard.putNumber("frontLeftAngle",frontLeftAngle);
            SmartDashboard.putNumber("pangle", panglefl);
            SmartDashboard.putNumber("fldv", fld.getOutputVoltage());
            SmartDashboard.putNumber("flrv", frd.getOutputVoltage());
            fls.set(-frontLeftAngle*.177777777);
            frs.set(-frontRightAngle*.177777777);
            bls.set(-rearLeftAngle*.177777777);
            brs.set(-rearRightAngle*.177777777);
//            if((panglefl-frontLeftAngle)<-180){
//            	double x=360+(panglefl-frontLeftAngle);
//            	double y=-(x-panglefl);
//            	SmartDashboard.putNumber("CAN", -y*.177777777);
//            	fls.set(-y*.177777777);
//            	fls.setPosition(frontLeftAngle*.177777777);
//            }
//            else if((panglefl-frontLeftAngle)>180){
//            	double x=360-(panglefl-frontLeftAngle);
//            	double y=(x+panglefl);
//            	fls.set(-y*.177777777);
//            	fls.setPosition(frontLeftAngle*.177777777);
//            }
//            else{
//            	fls.set(-frontLeftAngle*.177777777);
//            }
//            
//            if((panglefr-frontRightAngle)<-180){
//            	double x=360+(panglefr-frontRightAngle);
//            	double y=-(x-panglefr);
//            	frs.set(-y*.177777777);
//            	frs.setPosition(frontRightAngle*.177777777);
//            }
//            else if((panglefr-frontRightAngle)>180){
//            	double x=360-(panglefr-frontRightAngle);
//            	double y=(x+panglefr);
//            	frs.set(-y*.177777777);
//            	frs.setPosition(frontRightAngle*.177777777);
//            }
//            else{
//            	frs.set(-frontRightAngle*.177777777);
//            }
//            if((panglebl-rearLeftAngle)<-180){
//            	double x=360+(panglebl-rearLeftAngle);
//            	double y=-(x-panglebl);
//            	bls.set(-y*.177777777);
//            	bls.setPosition(rearLeftAngle*.177777777);
//            }
//            else if((panglebl-rearLeftAngle)>180){
//            	double x=360-(panglebl-rearLeftAngle);
//            	double y=(x+panglebl);
//            	bls.set(-y*.177777777);
//            	bls.setPosition(rearLeftAngle*.177777777);
//            }
//            else{
//            	bls.set(-rearLeftAngle*.177777777);
//            }
//            if((panglebr-rearRightAngle)<-180){
//            	double x=360+(panglebr-rearRightAngle);
//            	double y=-(x-panglebr);
//            	brs.set(-y*.177777777);
//            	brs.setPosition(rearRightAngle*.177777777);
//            }
//            else if((panglebr-rearRightAngle)>180){
//            	double x=360-(panglebr-rearRightAngle);
//            	double y=(x+panglebr);
//            	brs.set(-y*.177777777);
//            	brs.setPosition(rearRightAngle*.177777777);
//            }
//            else{
//            	brs.set(-rearRightAngle*.177777777);
//            }
            
            SmartDashboard.putNumber("fld", fldSpeed);
            SmartDashboard.putNumber("frd", frdSpeed);
            SmartDashboard.putNumber("bld", bldSpeed);
            SmartDashboard.putNumber("brd", brdSpeed);
            }
            if(fldSpeed>.3)
            fld.set(-fldSpeed);
            else
            	fld.set(0);
            if(frdSpeed>.3)
            frd.set(frdSpeed);
            else
            	frd.set(0);
            if(bldSpeed>.3)
            bld.set(-bldSpeed);
            else
            	bld.set(0);
            if(brdSpeed>.3)
            brd.set(brdSpeed);
            else
            	brd.set(0);
         panglefl=frontLeftAngle;
         panglefr=frontRightAngle;
         panglebl=rearLeftAngle;
         panglebr=rearRightAngle;
            
        }
    }
    public void test() {
    }
}
