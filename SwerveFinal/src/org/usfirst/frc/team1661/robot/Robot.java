package org.usfirst.frc.team1661.robot;
import edu.wpi.first.wpilibj.*;

public class Robot extends SampleRobot {
	   double width;
	   double length;
	   double R;
	   double LRscaleFactor;
	   double WRscaleFactor;
	   double RADIANS,NATIVE;
	   private Joystick XC=new Joystick(0);
		private CANTalon fld; 
		ADXRS450_Gyro g;
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
    	NATIVE=8/45;
    	g=new ADXRS450_Gyro();
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
    	g.calibrate();
    	fls.setPosition(0);
	    frs.setPosition(0);
	    brs.setPosition(0);
	    bls.setPosition(0);
	    double A,B,C,D;
	    double fldSpeed=0;
        double frdSpeed=0;
        double bldSpeed=0;
        double brdSpeed=0;
        double desiredFrontRightAngle=0,desiredFrontLeftAngle=0,desiredRearLeftAngle=0,desiredRearRightAngle=0;
        double currentFrontRightAngle=0,currentFrontLeftAngle=0,currentRearLeftAngle=0,currentRearRightAngle=0;
        double deltaFrontRightAngle,deltaFrontLeftAngle,deltaRearLeftAngle,deltaRearRightAngle;
        double setFrontRightAngle,setFrontLeftAngle,setRearLeftAngle,setRearRightAngle;
        while (isOperatorControl() && isEnabled()) {
        	double strafe = XC.getRawAxis(0);
            double fwd =-XC.getRawAxis(1);
            double rotation = XC.getRawAxis(4);
            if(Math.abs(XC.getRawAxis(0))<=.25){
            	strafe=0;
            }
            if(Math.abs(XC.getRawAxis(1))<=.25){
            	fwd=0;
            }
            if(Math.abs(XC.getRawAxis(4))<=.25){
            	rotation=0;
            }
            if(fwd!=0||strafe!=0||rotation!=0){
            	double fwdg=(fwd*Math.cos(Math.toRadians(g.getAngle())))+(strafe*Math.sin(Math.toRadians(g.getAngle())));
            	double strafeg=(-fwd*Math.sin(Math.toRadians(g.getAngle())))+(strafe*Math.cos(Math.toRadians(g.getAngle())));
                A = strafeg - rotation * LRscaleFactor;
                B = strafeg + rotation * LRscaleFactor;
                C = fwdg - rotation * WRscaleFactor;
                D = fwdg + rotation * WRscaleFactor;

                desiredFrontRightAngle= Math.atan2(B,C) * RADIANS;
                desiredFrontLeftAngle= Math.atan2(B,D) * RADIANS;
                desiredRearLeftAngle= Math.atan2(A,D) * RADIANS;
                desiredRearRightAngle= Math.atan2(A,C) * RADIANS;
               
                deltaFrontRightAngle=deltaNewAngle(Math.toRadians(desiredFrontRightAngle),Math.toRadians(currentFrontRightAngle));
                deltaFrontLeftAngle=deltaNewAngle(Math.toRadians(desiredFrontLeftAngle),Math.toRadians(currentFrontLeftAngle));
                deltaRearLeftAngle=deltaNewAngle(Math.toRadians(desiredRearLeftAngle),Math.toRadians(currentRearLeftAngle));
                deltaRearRightAngle=deltaNewAngle(Math.toRadians(desiredRearRightAngle),Math.toRadians(currentRearRightAngle));
               
                setFrontRightAngle=(currentFrontRightAngle+deltaFrontRightAngle);
                setFrontLeftAngle=(currentFrontLeftAngle+deltaFrontLeftAngle);
                setRearLeftAngle=(currentRearLeftAngle+deltaRearLeftAngle);
                setRearRightAngle=(currentRearRightAngle+deltaRearRightAngle);
                
                fls.set(-setFrontLeftAngle*NATIVE);
                frs.set(-setFrontRightAngle*NATIVE);
                bls.set(-setRearLeftAngle*NATIVE);
                brs.set(-setRearRightAngle*NATIVE);
                
                frdSpeed = Math.sqrt((B * B) + (C * C));
                fldSpeed = Math.sqrt((B * B) + (D * D));
                bldSpeed = Math.sqrt((A * A) + (D * D));
                brdSpeed = Math.sqrt((A * A) + (C * C));
                
                double maxSpeed = Math.max(fldSpeed, frdSpeed);
                maxSpeed = Math.max(maxSpeed, bldSpeed);
                maxSpeed = Math.max(maxSpeed, brdSpeed);
                
                if (maxSpeed > 1){
                   fldSpeed = fldSpeed / maxSpeed;
                   frdSpeed = frdSpeed / maxSpeed;
                   bldSpeed = bldSpeed / maxSpeed;
                   brdSpeed = brdSpeed / maxSpeed;
                } 
                if((desiredFrontLeftAngle-currentFrontLeftAngle)>Math.PI/2||(desiredFrontLeftAngle-currentFrontLeftAngle)<-Math.PI/2){
                	fldSpeed=-fldSpeed;
                }
                if((desiredFrontRightAngle-currentFrontRightAngle)>Math.PI/2||(desiredFrontRightAngle-currentFrontRightAngle)<-Math.PI/2){
                	frdSpeed=-frdSpeed;
                }
                if((desiredRearLeftAngle-currentRearLeftAngle)>Math.PI/2||(desiredRearLeftAngle-currentRearLeftAngle)<-Math.PI/2){
                	bldSpeed=-bldSpeed;
                }
                if((desiredRearRightAngle-currentRearRightAngle)>Math.PI/2||(desiredRearRightAngle-currentRearRightAngle)<-Math.PI/2){
                	brdSpeed=-brdSpeed;
                }
                fld.set(-fldSpeed);
            	frd.set(frdSpeed);
            	bld.set(-bldSpeed);
            	brd.set(brdSpeed);
                
                currentFrontRightAngle= setFrontRightAngle;
                currentFrontLeftAngle= setFrontLeftAngle;
                currentRearLeftAngle= setRearLeftAngle;
                currentRearRightAngle= setRearRightAngle; 
            }
            if(fwd==0&&strafe==0&&rotation==0){
            	fld.set(0);
            	frd.set(0);
            	bld.set(0);
            	brd.set(0);
            }
            

        }
    }
    public void test() {
    }
    public static double deltaNewAngle(double desired, double current){
    	double desiredAngle=desired;
    	double currentAngle=current;
    	double delta=desiredAngle-currentAngle;
    	if(delta>Math.PI){
    		delta=delta-2*Math.PI;
    	}
    	else if(delta<-Math.PI){
    		delta=delta+2*Math.PI;
    	}
    	else if(delta>Math.PI/2){
    		delta=delta-Math.PI;
    	}
    	else if(delta<-Math.PI/2){
    		delta=delta+Math.PI;
    	}
    	return Math.toDegrees(delta);
    }
}
