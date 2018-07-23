// By LI SHIDI, Jun 30, 2017
// Programming for KUKA robot manipulator. Allow the client to communicate with the robot via UDP protocol

package application;

import java.io.IOException;
import java.net.*; // socket
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.TimeUnit;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.geometricModel.World;

import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.*;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;
import com.kuka.generated.ioAccess.*;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a
 * {@link RoboticsAPITask#run()} method, which will be called successively in
 * the application lifecycle. The application will terminate automatically after
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an
 * exception is thrown during initialization or run.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the
 * {@link RoboticsAPITask#dispose()} method.</b>
 *
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class LfD_lishidi extends RoboticsAPIApplication
{
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private Tool ToolGripper;
	private Workpiece Pen;
	private CartesianImpedanceControlMode mode = null ;
	private GripperIOGroup FingerTip;

	Boolean _IsRunning = true;
	int state = 0;
	int grip = 0; // 1 stands for open and Pos(0), 0 stand for closed and Pos(255)

	byte[] receiveDataS = new byte[5];
	double[] gripper_hand = new double[2]; // Distance from gripper to left & right hands
    private UdpClient client = null;

    DatagramSocket  server ;
    byte[] recvBuf = new byte[2000];
    DatagramPacket recvPacket ;
    double frequency = 100;

    boolean done = false;
    boolean origin = false;
    boolean demo = false;
    boolean executing = false;
    int endOfTrajCount = 0;
	double RB1_x = 0.0 ;
	double RB1_y = 0.0 ;
	double RB1_z = 0.0 ;
	double RB1_oa = 0.0 ;
	double RB1_ob = 0.0 ;
	double RB1_og = 0.0 ;

	double Old_RB1_x = 0.0 ;
	double Old_RB1_y = 0.0 ;
	double Old_RB1_z = 0.0 ;
	double Old_RB1_oa = 0.0 ;
	double Old_RB1_ob = 0.0 ;
	double Old_RB1_og = 0.0 ;

	private ATIIOGroup ForceTorqueSensor ;

	public void initialize()
	{
		System.out.println("Initialization begins!" );
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getDevice(kuka_Sunrise_Cabinet_1, "LBR_iiwa_14_R820_1");

		lbr_iiwa_14_R820_1 = ServoMotionUtilities.locateLBR(getContext());
		kuka_Sunrise_Cabinet_1 = (Controller) getContext().getControllers().toArray()[0];
		FingerTip = new GripperIOGroup(kuka_Sunrise_Cabinet_1);
		ToolGripper = getApplicationData().createFromTemplate("ToolGripper");

		// setting stiffness parameters
		mode = new CartesianImpedanceControlMode() ;
		mode.parametrize(CartDOF.A).setStiffness(300);  	// rotation about Z
		mode.parametrize(CartDOF.B).setStiffness(300);		// rotation about Y
		mode.parametrize(CartDOF.C).setStiffness(300);		// rotation about X
		//mode.parametrize(CartDOF.ROT).setStiffness(0.1);    // rotation all range 0 - 300

		//mode.parametrize(CartDOF.TRANSL).setStiffness(4000.0);   // for all translation axis
		mode.parametrize(CartDOF.X).setStiffness(4000);  // range 0 - 5000
		mode.parametrize(CartDOF.Y).setStiffness(4000);
		mode.parametrize(CartDOF.Z).setStiffness(4000);
		mode.setNullSpaceStiffness(200);                      // null space range 0 - 200

		mode.parametrize(CartDOF.ALL).setDamping(0.7) ;
		mode.setNullSpaceDamping(0.7);

		ForceTorqueSensor = new ATIIOGroup(kuka_Sunrise_Cabinet_1);
		System.out.println("Initialization ends!" );
	}

	@Override
	public void dispose()
	{
		_IsRunning = false ;
		System.out.println(" Closing sockets in Dispose Block");

		if(client != null)
			client.kill();
		if(server != null)
			server.close();
		super.dispose();
	}

	private static double getMax(double[] arr)
	{
		double max = arr[0];
		for(int i = 1; i < arr.length; i++)
		{
			if(arr[i] > max)
			{
				max = arr[i];
			}
		}
		return max;
	}

	private static double getMin(double[] arr)
	{
		double min = arr[0];
		for(int i = 1; i < arr.length; i++)
		{
			if(arr[i] < min)
			{
				min = arr[i];
			}
		}
		return min;
	}

	private static double getSqrSumSqrt(double[] arr)
	{
		double result = 0.0;
		for(int i = 0; i < arr.length; i++)
		{
			result = result + arr[i] * arr[i];
		}
		return Math.sqrt(result);
	}

	private void moveToInitialPosition()
    {
		System.out.println("Run into moveToInitialPosition()!");
		lbr_iiwa_14_R820_1.move(ptp(Math.toRadians(40.22), Math.toRadians(49.19), Math.toRadians(0.80), Math.toRadians(-83.60),
        		Math.toRadians(120.23), Math.toRadians(61.71), Math.toRadians(-85.77)).setJointVelocityRel(0.3));

		if (FingerTip.getActReq() != 9)
    	{
    		FingerTip.setActReq(9); // gripper closed
    	}

    	FingerTip.setSpeed(255);
		FingerTip.setForce(5);

        FingerTip.setPosReq(255); // close the gripper
        ThreadUtil.milliSleep(2000); // wait for 2 seconds

        System.out.println("Finish moveToInitialPosition()!");
    }

	public void run()
	{
		System.out.println("Setting the KeyBar event!");
		IUserKeyBar keyBar = getApplicationUI().createUserKeyBar("LfD");
		IUserKeyListener Listener = new IUserKeyListener()
		{
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event)
			{
				// TODO Auto-generated method stub
		        if(event == UserKeyEvent.FirstKeyDown)
		        {
		        	getLogger().info("Go to origin");
		        	origin = true;
		        }
		        if(event == UserKeyEvent.SecondKeyDown)
		        {
		        	getLogger().info("Move to initial position");
		        	moveToInitialPosition();
		        }
		    }
	    };

	    IUserKey Key = keyBar.addDoubleUserKey(0, Listener, true);
	    Key.setText(UserKeyAlignment.TopMiddle, "Origin");
	    Key.setText(UserKeyAlignment.BottomMiddle, "Init");
	    keyBar.publish(); // end of setting KeyBar

	    System.out.println("Run!");

	    try
	    {
	    	server = new DatagramSocket(12358);
	       	recvPacket  = new DatagramPacket(recvBuf , recvBuf.length);

		  	FingerTip.setActReq(9); // gripper close
			FingerTip.setSpeed(120);
			FingerTip.setForce(30);
			ToolGripper.attachTo(lbr_iiwa_14_R820_1.getFlange());
			moveToInitialPosition();

			if (origin)
			{
				FingerTip.setActReq(11); // finger touches finger
				ToolGripper.getFrame("/BasePad").move(ptp(getApplicationData().getFrame("/tableOrigin/P1")).setJointVelocityRel(0.1));
				// the frame "/tableOrigin/P1" is pre-set in the software to make the objects share the same coordinate with the end effector of the robot
			}

			/*SmartServo aSmartServoMotion = new SmartServo(lbr_iiwa_14_R820_1.getCurrentJointPosition());
			aSmartServoMotion.useTrace(true);

	        // for Automatic mode 0.25, for T1 mode 1
	        aSmartServoMotion.setJointAccelerationRel(1.0);
	        aSmartServoMotion.setJointVelocityRel(1.0);
	        aSmartServoMotion.setMinimumTrajectoryExecutionTime(5e-3);

	        System.out.println("SmartServo: Starting Realtime Motion in Position Mode");

  			if (aSmartServoMotion.validateForImpedanceMode(ToolGripper) != true)
  			{
  			    getLogger().info("Validation for SmartServo Compliant control failed");
  			}

			ToolGripper.getFrame("/BasePad").moveAsync(aSmartServoMotion.setMode(mode));
			ISmartServoRuntime theServoRuntime = aSmartServoMotion.getRuntime();*/

			/********LIN*********/
			SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad")));
	        aSmartServoLINMotion.useTrace(true);

	        // set max velocity and acceleration in Cartesian space
	        double maxTranslationVelocity[] = {750,500,750};
			aSmartServoLINMotion.setMaxTranslationVelocity(maxTranslationVelocity);
			double maxTranslationAcceleration[] = {5000,4000,5000};
			aSmartServoLINMotion.setMaxTranslationAcceleration(maxTranslationAcceleration);
			double maxOrientationVelocity[] = {350,350,350};
			aSmartServoLINMotion.setMaxOrientationVelocity(maxOrientationVelocity);
			double maxOrientationAcceleration[] = {250,250,250};
			aSmartServoLINMotion.setMaxOrientationAcceleration(maxOrientationAcceleration);

			aSmartServoLINMotion.setMaxNullSpaceVelocity(100);
			aSmartServoLINMotion.setMaxNullSpaceAcceleration(100);
			aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

			System.out.println("SmartServoLIN: Starting Realtime Motion in Position Mode");

			if (aSmartServoLINMotion.validateForImpedanceMode(ToolGripper) != true)
  			{
  			    getLogger().info("Validation for SmartServo Compliant control failed");
  			}

			ToolGripper.getFrame("/BasePad").moveAsync(aSmartServoLINMotion.setMode(mode));
			ISmartServoLINRuntime theServoRuntime = aSmartServoLINMotion.getRuntime();
			/********LIN*********/

			Frame goalFrame = new Frame();
			Frame currentFrame = new Frame();
			double goalVelocity[] = new double[3];

			// get goal Frame for the first time
			goalFrame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad"),getApplicationData().getFrame("/tableOrigin"));//getFrame("/BasePad"));
			ForceSensorData torData = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getRootFrame());//getFrame("/BasePad"));

			double ForceX = torData.getForce().getX();
			double ForceY = torData.getForce().getY();
			double ForceZ = torData.getForce().getZ();

			System.out.println("Go into the while" );

			while(!done)
			{
				//System.out.println("Before receive!");
		        server.receive(recvPacket);
		        //System.out.println("Receive!" );
		        String recvStr = new String(recvPacket.getData() , 0 , recvPacket.getLength());

		        //System.out.println("Receive string: " + recvStr);
		        String[] splitStr = recvStr.split(" ");
		        //System.out.printf("Receive length data: %d\n" , splitStr.length);
		        String sendStr = "" ;
		        byte[] sendBuf;
		        double[] recvDouble = new double[splitStr.length];

		        for(int j=0;j<splitStr.length;j++)
		        {
		          //System.out.println("Hello World!" + jointStr[j]);
		          recvDouble[j] = Double.valueOf(splitStr[j]);
		          //System.out.printf("receive float data:%f  \n" , recvDouble[j]);
		        }

		      //if grip changes
		        if (grip != (int)recvDouble[1])
		        {
		        	grip = (int)recvDouble[1];
		        	if (grip == 1)
		        		FingerTip.setPosReq(0); // open
		        	else
		        		FingerTip.setPosReq(255); // close
		        }

		        // if state changes
		        if (state != (int)recvDouble[0])
		        {
		        	state = (int)recvDouble[0];
		        	switch (state)
		        	{
		        	case 1: // executing, and also the initial value
		        		demo = false;
		        		mode.parametrize(CartDOF.X).setStiffness(30);
		        		mode.parametrize(CartDOF.Y).setStiffness(30);
		        		theServoRuntime.changeControlModeSettings(mode);
		        		ThreadUtil.milliSleep(2000); // wait for 2 seconds

		        		mode.parametrize(CartDOF.X).setStiffness(200);
		        		mode.parametrize(CartDOF.Y).setStiffness(200);
		        		mode.setNullSpaceStiffness(200);
		        		theServoRuntime.changeControlModeSettings(mode);
		        		ThreadUtil.milliSleep(2000); // wait for 2 seconds

		        		mode.parametrize(CartDOF.X).setStiffness(1000);
		        		mode.parametrize(CartDOF.Y).setStiffness(1000);
		        		theServoRuntime.changeControlModeSettings(mode);
		        		ThreadUtil.milliSleep(2000); // wait for 2 seconds

		        		mode.parametrize(CartDOF.X).setStiffness(4000);
		        		mode.parametrize(CartDOF.Y).setStiffness(4000);
		        		theServoRuntime.changeControlModeSettings(mode);
		        		ThreadUtil.milliSleep(500); // wait for 0.5 seconds
		        		break;

		        	case 2: // record command
		        		break;

		        	case 3: // teaching
		        		demo = true;
		        		mode.parametrize(CartDOF.X).setStiffness(0);
		        		mode.parametrize(CartDOF.Y).setStiffness(0);
		        		//mode.parametrize(CartDOF.Z).setStiffness(0);
		        		mode.setNullSpaceStiffness(0);
		        		theServoRuntime.changeControlModeSettings(mode);
		        		break;

		        	case 4:
		        		break;

		        	case 5:
		        		break;

		        	case 6: // go back to initial point after executing one trajectory
		        	    moveToInitialPosition();
		        		break;

		        	case 7: // relax all
		        		demo = true;
		        		mode.parametrize(CartDOF.ROT).setStiffness(0);
		        		mode.parametrize(CartDOF.TRANSL).setStiffness(0);
		        		mode.setNullSpaceStiffness(0);
		        		theServoRuntime.changeControlModeSettings(mode);
		        		break;

                    case 8: // relax null space
		        		demo = true;
		        		mode.setNullSpaceStiffness(0);
		        		theServoRuntime.changeControlModeSettings(mode);
		        		break;

                    case 9: // tighten null space
		        		demo = true;
		        		mode.setNullSpaceStiffness(200);
		        		theServoRuntime.changeControlModeSettings(mode);
		        		break;

		        	default:
		        		break;
		        	}
		        }

				// set the destination
				if (demo)
				{
					theServoRuntime.setDestination(goalFrame);
				}
				else
				{
					if(recvDouble.length >= 8)
					{
						// set next goal point
						goalFrame.setX(recvDouble[2]);
						goalFrame.setY(recvDouble[3]);
						goalFrame.setZ(recvDouble[4]);
						goalFrame.setAlphaRad(recvDouble[5]);
						goalFrame.setBetaRad(recvDouble[6]);
						goalFrame.setGammaRad(recvDouble[7]);

						// calculate the target speed
						goalVelocity[0] = Math.abs(goalFrame.getX() - Old_RB1_x) * frequency;
						goalVelocity[1] = Math.abs(goalFrame.getY() - Old_RB1_y) * frequency;
						goalVelocity[2] = Math.abs(goalFrame.getZ() - Old_RB1_z) * frequency;

						// execute
						theServoRuntime.setDestination(goalFrame);
						/*if(getSqrSumSqrt(goalVelocity) > 1000 || getSqrSumSqrt(goalVelocity) < 120)
                        {
                            theServoRuntime.setDestination(goalFrame);
                        }
                        else
                        {
                            theServoRuntime.setDestination(goalFrame, goalVelocity);
                        }*/

						// save the old point to Old_RB1
						Old_RB1_x = goalFrame.getX();
						Old_RB1_y = goalFrame.getY();
						Old_RB1_z = goalFrame.getZ();
						endOfTrajCount = 0;
						executing = true;
					}
				}

				torData = lbr_iiwa_14_R820_1.getExternalForceTorque(ToolGripper.getRootFrame());
    			//ForceX = torData.getForce().getX();
    			//ForceY = torData.getForce().getY();
    			//ForceZ = torData.getForce().getZ();
				currentFrame = lbr_iiwa_14_R820_1.getCurrentCartesianPosition(ToolGripper.getFrame("/BasePad"),getApplicationData().getFrame("/tableOrigin"));//getFrame("/BasePad"));
    			RB1_x = currentFrame.getX();
    			RB1_y = currentFrame.getY();
    			RB1_z = currentFrame.getZ();
    			RB1_oa = currentFrame.getAlphaRad();
    			RB1_ob = currentFrame.getBetaRad();
    			RB1_og = currentFrame.getGammaRad();
    			int fingerForce = FingerTip.getFingerPos();
//    			System.out.println("Get current info, before sending!");

    			sendStr = "";
    			sendStr = sendStr.valueOf(grip)+" "+sendStr.valueOf(RB1_x)+" "+sendStr.valueOf(RB1_y)+" "+sendStr.valueOf(RB1_z)+" "+sendStr.valueOf(RB1_oa)+" "+sendStr.valueOf(RB1_ob)+" "+sendStr.valueOf(RB1_og)+" "+sendStr.valueOf(fingerForce);
                //System.out.printf("sendStr = %s\n" ,sendStr);

                sendBuf = sendStr.getBytes();
                DatagramPacket sendPacket
                    = new DatagramPacket(sendBuf , sendBuf.length , recvPacket.getAddress() , recvPacket.getPort() );
                server.send(sendPacket);
//                System.out.println("sent Packet!");
			}

			System.out.println("The end!");
	    }

	    //error catch
	    catch(Exception e)
	    {
		      e.printStackTrace();
		}
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args)
	{
		LfD_lishidi app = new LfD_lishidi();
		app.runApplication();
	}
}
