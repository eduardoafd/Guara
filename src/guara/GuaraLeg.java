package guara;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.UniversalJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.Robot;
import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class GuaraLeg {
	
	public UniversalJoint HipJoint;
	public PinJoint KneeJoint;
	public PinJoint AnkleJoint;
	public String name;
	public GuaraRobot rob;
	public boolean isFlying = false;
	private ArrayList<GroundContactPoint> gcPoints = new ArrayList<GroundContactPoint>();
	public GroundContactPoint gcHeel, gcToe;
	
	//constructor
	public GuaraLeg(UniversalJoint HipJoint, PinJoint KneeJoint, PinJoint AnkleJoint, String name) {
		this.HipJoint = HipJoint;
		this.KneeJoint = KneeJoint;
		this.AnkleJoint = AnkleJoint;
		this.name = name;
		this.rob = (GuaraRobot) HipJoint.getRobot();
		this.isFlying = false;
		this.gcPoints = getGCPs();
		this.gcHeel = gcPoints.get(0);
		this.gcToe = gcPoints.get(1);
	}
	
	private ArrayList<GroundContactPoint> getGCPs() {
		ArrayList<GroundContactPoint> gcs = new ArrayList<GroundContactPoint>();
		AnkleJoint.recursiveGetAllGroundContactPoints(gcs);
		
		return gcs;
	}
	
	void InverseKinematics(double []vector, double []angles) {
		//O VETOR POSICAO DADO COMO ENTRADA DEVE ESTAR NO SISTEMA DE COORDENADAS DA SIMULACAO, SENDO:
		//				  ________ Y
		//				 /\		    \	
		//				/  \	     \
		//			   /	\________ \
		//			   \	/		   \--------------->X
		//				\_ /		   |
		//				  /		       |
		//				  \		       |
		//				   \		   |
		//					\__	       |
		//					PERNA2     V	
		
		 //O vetor angles eh o vetor que carregara os angulos de junta
		 //O vetor vector eh o vetor posicao do end efector (ankleJoint) a partir do abdHip
		 // Sendo 0,1,2 respectivamente 
		   
		   double l2 = Math.pow(vector[0], 2) + Math.pow(vector[1], 2) + Math.pow(vector[2], 2);
		   
		   angles[0] =- Math.atan(vector[1]/vector[2]);
		   
		   angles[2] = Math.acos( ( l2 - Math.pow(rob.lThighZ, 2) - Math.pow(rob.lShankZ, 2) ) / (2*rob.lThighZ*rob.lShankZ) );
		   
		   
		   angles[1] = Math.asin(rob.lShankZ*Math.sin(angles[2])/(Math.sqrt(l2))) - Math.atan(vector[0]/Math.sqrt(Math.pow(vector[1], 2)+Math.pow(vector[2], 2)));
		   
		   angles[2] = - angles[2];
		   
	
	}
	
	
 }




