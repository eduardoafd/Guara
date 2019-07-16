package guara;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.UniversalJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import java.util.ArrayList;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;

public class GuaraController implements RobotController{
	
   private final YoVariableRegistry registry = new YoVariableRegistry("guaraController");

   private GuaraKinematics kinematics;
   public GuaraWaveGait waveGait;
   public GuaraRobot rob;

//Constantes utilizadas no controle Estatico e Quase-estatico da perna
	double k1,k2,k3,k4;
	double kd1, kd2, kd3, kd4;
	double kKnee,kdKnee,kHip, kdHip;
	
// ---------------------------------------------------------------------------	

//Variaveis utilizadas no controle de posicao da perna
	FloatingJoint rootJoint;
	UniversalJoint abdFlexHip0 , abdFlexHip1, abdFlexHip2, abdFlexHip3;
	PinJoint flexKnee0, flexKnee1, flexKnee2, flexKnee3;
	PinJoint flexAnkle0, flexAnkle1, flexAnkle2, flexAnkle3, abdHip2;
	
	private final FrameVector3D auxVector = new FrameVector3D();
	private double posicaoPerna[] =new double[3];
	private final YoDouble posicaoX = new YoDouble("posicaoX", registry);
	private final YoDouble posicaoY = new YoDouble("posicaoY", registry);
	private final YoDouble posicaoZ = new YoDouble("posicaoZ", registry);
	private final YoDouble erro_qKnee = new YoDouble("erro_qKnee", registry);
	private final YoDouble erro_qdKnee = new YoDouble("erro_qdKnee", registry);
	private final YoDouble erro_qHip = new YoDouble("erro_qHip", registry);
	private final YoDouble erro_qdHip = new YoDouble("erro_qdHip", registry);
			
	private double numberOfSetPoints = 16;
	
//Variaveis utilizadas na iniciacao da andadura
	private boolean initiateGait = true;
	private double setPoints1[][] = new double[(int) numberOfSetPoints][3];		//SETPOINTS DO PRIMEIRO VOO DA PATA (2)
	private double setPoints2[][] = new double[(int) numberOfSetPoints][3];		//SETPOINTS DO SEGUNDO VOO DA PATA (3)
	private double setPoints3[][] = new double[(int) numberOfSetPoints][3];
	private double staticLeg0Angles[] = new double[3];
	private double staticLeg1Angles[] = new double[3];
	private double staticLeg2Angles[] = new double[3];
	private double staticLeg3Angles[] = new double[3];
	
	
	private double angulos[] = new double[3];
	private double hipToAnkle2[] = new double[3];
	private double flyingLegAngles[] = new double[4]; //vetor que recebe os angulos da cinematica inversa
	public double stepSize = 0.05, range;
	
	
	private YoBoolean gcp0 = new YoBoolean("GCP0", registry);	//INFORMA SE O GCP FOI DEVIDAMENTE DESABILITADO
	private YoBoolean gcp1 = new YoBoolean("GCP1", registry);	//INFORMA SE O GCP FOI DEVIDAMENTE DESABILITADO
	private YoBoolean gcp2 = new YoBoolean("GCP2", registry);	//INFORMA SE O GCP FOI DEVIDAMENTE DESABILITADO
	private YoBoolean gcp3 = new YoBoolean("GCP3", registry);	//INFORMA SE O GCP FOI DEVIDAMENTE DESABILITADO
	
	boolean nloopsReceived = false;
	int firstNLoops = 0;
	boolean calculouFA = false;
	double forwardAngles[] = new double[3];
	double forwardVector[] = new double[3];
	boolean isGCDisabled = false;
	
	int numberOfLoops = 0;
	int setPointDuration = 1000;
	int leanHipDuration = 3000;
	int forwardDuration = 5000;
	int currentSetPoint = 0;
	
	private final YoDouble nloops = new YoDouble("nloops", registry);
	private final YoDouble legAngle1 = new YoDouble("legAngle1", registry);
	private final YoDouble legAngle2 = new YoDouble("legAngle2", registry);
	private final YoDouble currentSP = new YoDouble("currentSP", registry);
	
//Variaveis usadas na continuacao da andadura----------------------------------
	private double legOffSet[] = {3, 0, 2, 1};
	private double setPointsKW23[][] = new double[(int) numberOfSetPoints][3];	//SetPoints definidos para voo das patas 2 e 3.
	private double setPointsKW01[][] = new double[(int) numberOfSetPoints][3];	//SetPoints definidos para voo das patas 0 e 1.
	private boolean finishedStep = false;
	private boolean moveLegLR = false;
	private boolean notMovingForward = true;
	private int staticNloops, spDuration = 1000, sp;
//-----------------------------------------------------------------------------	
	
	private YoDouble tau_abdHip0, tau_flexHip0, tau_abdHip1, tau_flexHip1, tau_abdHip2, tau_flexHip2, tau_abdHip3, tau_flexHip3;
	private YoDouble q_abdHip0, q_flexHip0, q_abdHip1, q_flexHip1, q_abdHip2, q_flexHip2, q_abdHip3, q_flexHip3;
	private YoDouble qd_abdHip0, qd_flexHip0, qd_abdHip1, qd_flexHip1, qd_abdHip2, qd_flexHip2, qd_abdHip3, qd_flexHip3;

	private YoDouble tau_flexKnee0,tau_flexKnee1,tau_flexKnee2,tau_flexKnee3;
	private YoDouble q_flexKnee0,q_flexKnee1,q_flexKnee2,q_flexKnee3, qd_flexKnee0,qd_flexKnee1,qd_flexKnee2,qd_flexKnee3;
	
	private YoDouble tau_flexAnkle0, tau_flexAnkle1, tau_flexAnkle2, tau_flexAnkle3;
	private YoDouble q_flexAnkle0, q_flexAnkle1, q_flexAnkle2, q_flexAnkle3, qd_flexAnkle0, qd_flexAnkle1, qd_flexAnkle2, qd_flexAnkle3;
	
	private GuaraLeg[] pernas;
	
	GroundContactPoint gcToe0, gcToe1, gcToe2, gcToe3;
	GroundContactPoint gcHeel0, gcHeel1, gcHeel2, gcHeel3;
	ArrayList<GroundContactPoint> gcPoints = new ArrayList<GroundContactPoint>();
	
//----------------------------------------------------------------------------
	private Point3D tempCOMPosition = new Point3D(); //posicao do centro de massa
	private double vetorCOM[] = new double[3];
	private final YoFramePoint COM = new YoFramePoint("COM", ReferenceFrame.getWorldFrame(), registry);
	private final YoFrameVector ComPositionVector = new YoFrameVector("ComPositionVector", ReferenceFrame.getWorldFrame(), registry);
	private final YoFrameVector linearMomentum = new YoFrameVector("linearMomentum", ReferenceFrame.getWorldFrame(), registry);
	private final YoFrameVector angularMomentum = new YoFrameVector("angularMomentum", ReferenceFrame.getWorldFrame(), registry);
	private final YoFrameVector comVelocity = new YoFrameVector("comVelocity", ReferenceFrame.getWorldFrame(), registry);
	Vector3D ComPositionVector3D = new Vector3D();
	private YoDouble x = new YoDouble("x", registry);
	private YoDouble y = new YoDouble("y", registry);
	private YoDouble z = new YoDouble("z", registry);
	Graphics3DObject centroDM = new Graphics3DObject();
	
	// --------------------VARIAVEIS PARA OBTER DADOS DE POSICAO DO COM
	private YoDouble x_ankle0 = new YoDouble("x_ankle0", registry);  private YoDouble y_ankle0 = new YoDouble("y_ankle0", registry);	private YoDouble z_ankle0 = new YoDouble("z_ankle0", registry);
	private YoDouble x_ankle1 = new YoDouble("x_ankle1", registry);  private YoDouble y_ankle1 = new YoDouble("y_ankle1", registry);	private YoDouble z_ankle1 = new YoDouble("z_ankle1", registry);
	private YoDouble x_ankle2 = new YoDouble("x_ankle2", registry);  private YoDouble y_ankle2 = new YoDouble("y_ankle2", registry);	private YoDouble z_ankle2 = new YoDouble("z_ankle2", registry);
	private YoDouble x_ankle3 = new YoDouble("x_ankle3", registry);  private YoDouble y_ankle3 = new YoDouble("y_ankle3", registry);	private YoDouble z_ankle3 = new YoDouble("z_ankle3", registry);
	
	//---------------------------------------------------------------------------  
	
	
	//Contadores
	int i = 0, j = 0;
	
	
	
	Vector3D anklePos = new Vector3D(0.0, 0.0, 0.0);
	
	
	
	public GuaraController(GuaraRobot robot, YoGraphicsListRegistry yoGraphicsListRegistries){  //, String name) {
		this.rob = robot;
		
		this.rootJoint = (FloatingJoint) robot.getRootJoint();
		
		this.abdFlexHip0 = (UniversalJoint) robot.getAbdFlexHip0();
		this.abdFlexHip1 = (UniversalJoint) robot.getAbdFlexHip1();
		this.abdFlexHip2 = (UniversalJoint) robot.getAbdFlexHip2();
		this.abdFlexHip3 = (UniversalJoint) robot.getAbdFlexHip3();
		
		this.flexKnee0 = (PinJoint) robot.getFlexKnee0();
		this.flexKnee1 = (PinJoint) robot.getFlexKnee1();
		this.flexKnee2 = (PinJoint) robot.getFlexKnee2();
		this.flexKnee3 = (PinJoint) robot.getFlexKnee3();
		
		this.flexAnkle0 = (PinJoint) robot.getFlexAnkle0();
		this.flexAnkle1 = (PinJoint) robot.getFlexAnkle1();
		this.flexAnkle2 = (PinJoint) robot.getFlexAnkle2();
		this.flexAnkle3 = (PinJoint) robot.getFlexAnkle3();
		
		/*this.tau_abdHip0 = abdFlexHip0.getTauYoVariable();*/
		
		this.tau_abdHip0 = (YoDouble) robot.getVariable("tau_abdHip0");
		this.tau_abdHip1 = (YoDouble) robot.getVariable("tau_abdHip1");
		this.tau_abdHip2 = (YoDouble) robot.getVariable("tau_abdHip2");
		this.tau_abdHip3 = (YoDouble) robot.getVariable("tau_abdHip3");
		this.q_abdHip0 = (YoDouble) robot.getVariable("q_abdHip0");	
		this.q_abdHip1 = (YoDouble) robot.getVariable("q_abdHip1");
		this.q_abdHip2 = (YoDouble) robot.getVariable("q_abdHip2");
		this.q_abdHip3 = (YoDouble) robot.getVariable("q_abdHip3");
		this.qd_abdHip0 = (YoDouble) robot.getVariable("qd_abdHip0");	
		this.qd_abdHip1 = (YoDouble) robot.getVariable("qd_abdHip1");
		this.qd_abdHip2 = (YoDouble) robot.getVariable("qd_abdHip2");
		this.qd_abdHip3 = (YoDouble) robot.getVariable("qd_abdHip3");
		
		this.tau_flexHip0 = (YoDouble) robot.getVariable("tau_flexHip0");
		this.tau_flexHip1 = (YoDouble) robot.getVariable("tau_flexHip1");
		this.tau_flexHip2 = (YoDouble) robot.getVariable("tau_flexHip2");
		this.tau_flexHip3 = (YoDouble) robot.getVariable("tau_flexHip3");
		this.q_flexHip0 = (YoDouble) robot.getVariable("q_flexHip0");	
		this.q_flexHip1 = (YoDouble) robot.getVariable("q_flexHip1");
		this.q_flexHip2 = (YoDouble) robot.getVariable("q_flexHip2");
		this.q_flexHip3 = (YoDouble) robot.getVariable("q_flexHip3");
		this.qd_flexHip0 = (YoDouble) robot.getVariable("qd_flexHip0");	
		this.qd_flexHip1 = (YoDouble) robot.getVariable("qd_flexHip1");
		this.qd_flexHip2 = (YoDouble) robot.getVariable("qd_flexHip2");
		this.qd_flexHip3 = (YoDouble) robot.getVariable("qd_flexHip3");
						
		this.tau_flexKnee0 = (YoDouble) robot.getVariable("tau_flexKnee0");
		this.tau_flexKnee1 = (YoDouble) robot.getVariable("tau_flexKnee1");
		this.tau_flexKnee2 = (YoDouble) robot.getVariable("tau_flexKnee2");
		this.tau_flexKnee3 = (YoDouble) robot.getVariable("tau_flexKnee3");
		this.q_flexKnee0 = (YoDouble) robot.getVariable("q_flexKnee0");
		this.q_flexKnee1 = (YoDouble) robot.getVariable("q_flexKnee1");
		this.q_flexKnee2 = (YoDouble) robot.getVariable("q_flexKnee2");
		this.q_flexKnee3 = (YoDouble) robot.getVariable("q_flexKnee3");
		this.qd_flexKnee0 = (YoDouble) robot.getVariable("qd_flexKnee0");
		this.qd_flexKnee1 = (YoDouble) robot.getVariable("qd_flexKnee1");
		this.qd_flexKnee2 = (YoDouble) robot.getVariable("qd_flexKnee2");
		this.qd_flexKnee3 = (YoDouble) robot.getVariable("qd_flexKnee3");
		
		this.tau_flexAnkle0 = (YoDouble) robot.getVariable("tau_flexAnkle0");
		this.tau_flexAnkle1 = (YoDouble) robot.getVariable("tau_flexAnkle1");
		this.tau_flexAnkle2 = (YoDouble) robot.getVariable("tau_flexAnkle2");
		this.tau_flexAnkle3 = (YoDouble) robot.getVariable("tau_flexAnkle3");
		this.q_flexAnkle0 = (YoDouble) robot.getVariable("q_flexAnkle0");
		this.q_flexAnkle1 = (YoDouble) robot.getVariable("q_flexAnkle1");
		this.q_flexAnkle2 = (YoDouble) robot.getVariable("q_flexAnkle2");
		this.q_flexAnkle3 = (YoDouble) robot.getVariable("q_flexAnkle3");
		this.qd_flexAnkle0 = (YoDouble) robot.getVariable("qd_flexAnkle0");
		this.qd_flexAnkle1 = (YoDouble) robot.getVariable("qd_flexAnkle1");
		this.qd_flexAnkle2 = (YoDouble) robot.getVariable("qd_flexAnkle2");
		this.qd_flexAnkle3 = (YoDouble) robot.getVariable("qd_flexAnkle3");
		
		gcp0.set(true);
		gcp1.set(true);
		gcp2.set(true);
		gcp3.set(true);		

		
		//Zerando Matriz de setpoints
		for(i=0;i < (int)numberOfSetPoints;i++) {
			for(j = 0; j<3; j++){
				this.setPoints1[i][j] = 0;
			}
		}
		
		this.gcPoints = robot.getAllGroundContactPoints();
		this.gcHeel0 = gcPoints.get(0);
		this.gcToe0 = gcPoints.get(1);
		this.gcHeel1 = gcPoints.get(2);
		this.gcToe1 = gcPoints.get(3);
		this.gcHeel2 = gcPoints.get(4);
		this.gcToe2 = gcPoints.get(5);
		this.gcHeel3 = gcPoints.get(6);	
		this.gcToe3 = gcPoints.get(7);
				
		
		this.pernas = (GuaraLeg[]) robot.getLegs();
		
		

	    boolean drawCenterOfMass = true;
	    if (drawCenterOfMass){
	         YoGraphicPosition comPositionYoGraphic = new YoGraphicPosition("CoM", COM, 0.01, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);
	         yoGraphicsListRegistries.registerYoGraphic("allGraphics", comPositionYoGraphic);
	         yoGraphicsListRegistries.registerArtifact("allGraphics", comPositionYoGraphic.createArtifact());

	    }
	    
	    abdFlexHip0.getXYZToWorld(x, y, z);
	    
		initControl();
		
		
		
	}
	
	
	public void initControl() {
		staticLeg0Angles[0] = -(Math.PI/20); 	staticLeg0Angles[1] = rob.phiY;		staticLeg0Angles[2] = -rob.theta;
		staticLeg1Angles[0] = -(Math.PI/20);	staticLeg1Angles[1] = rob.phiY;		staticLeg1Angles[2] = -rob.theta;
		staticLeg2Angles[0] = -(Math.PI/20);	staticLeg2Angles[1] = rob.phiY;		staticLeg2Angles[2] = -rob.theta;
		staticLeg3Angles[0] = -(Math.PI/20);	staticLeg3Angles[1] = rob.phiY;		staticLeg3Angles[2] = -rob.theta;
		
		gcToe0.disable();
		gcToe1.disable();
		gcToe2.disable();
		gcToe3.disable();
		
		//Obtendo o vetor posicao da ankleJoint em relacao a hipJoint.
		//Passando Coordenadas da Junta para vetor hipToAnkle2
		hipToAnkleVector(hipToAnkle2, flexAnkle2, abdFlexHip2);
		
		inverseKinematics(staticLeg0Angles, hipToAnkle2);
		staticLeg1Angles[0] = staticLeg0Angles[0];			staticLeg1Angles[0] = staticLeg0Angles[0];			staticLeg1Angles[0] = staticLeg0Angles[0];
		staticLeg2Angles[0] = staticLeg0Angles[0];			staticLeg2Angles[0] = staticLeg0Angles[0];			staticLeg2Angles[0] = staticLeg0Angles[0];
		staticLeg3Angles[0] = staticLeg0Angles[0];			staticLeg3Angles[0] = staticLeg0Angles[0];			staticLeg3Angles[0] = staticLeg0Angles[0];
		
		//CALCULO DOS VETORES POSICAO DE CADA UM DOS SETPOINTS DA TRAJETORIA HEXAGOnal				
		defineSetPoints1(setPoints1, hipToAnkle2,stepSize, numberOfSetPoints);	
		
		defineSetPoints2(setPoints2, setPoints1);
		

		defineSetPoints3(setPoints3, setPoints1);
		
		defineSetPointsKW(setPointsKW23, setPointsKW01, setPoints1);
		//trajParabol(setPoints, hipToAnkle2, stepSize, numberOfSetPoints);
		
		//System.out.println("-----------------------SETPOINTS 2 -------------------");
		/*int i;
		for(i =0; i< (int) numberOfSetPoints; i++) {
			System.out.println("SetPoint " + i + ":" + "[ " + setPoints2[i][0] + ", " + setPoints2[i][1] + ", " + setPoints2[i][2] + "]");
		}
		System.out.println("-----------------------SETPOINTS 2 -------------------");
		*/
		
		
	}

	

   


	private void defineSetPointsKW(double[][] setPointsKW23, double[][] setPointsKW01, double[][] ref_setPoints) {
			//A matriz setPointsKW23 guarda os setpoints para as patas 2 e 3 enquanto a matriz setPointsKW01 guarda os setpoins para pernas 0 e 1
			//Vamos partir do setPoint 7 dos setPoints 1, e a partir dele calcular todos os setPoints anteriores.
			//Sabemos que o setPoint final da perna 3 eh [-0.05, -0.04432383389743616, -0.23847004473511105]
			//Variaveis para interpolacao linar: z = a*x + b
			double b, a, dx, dz;
			//Passando valores do setpoint1 para o setpoint2		
			int i;
			for(i = 0; i< (int)numberOfSetPoints; i++) {
				setPointsKW23[i][0] = ref_setPoints[i][0];	setPointsKW23[i][1] = ref_setPoints[i][1];	setPointsKW23[i][2] = ref_setPoints[i][2];
			}
			
			//Passando o valor do primeiro setpoint, que tambem eh setpoint usado para calcular os angulos de junta para o movimento horizontal do corpo.
			setPointsKW23[0][0] = -3*stepSize;	setPointsKW23[0][1] = -0.04432383389743616;	setPointsKW23[0][2] = -0.23847004473511105;
			
			
			//Calculo dos parametros da Interpolacao linear.
			dx = ref_setPoints[6][0] - setPointsKW23[0][0];
			dz = ref_setPoints[6][2] - setPointsKW23[0][2];
			a = dz/dx;
			b = ref_setPoints[6][2] - ref_setPoints[6][0]*a;
			
			//Calculo dos setpoints, posicoes 1 ate 5 na matriz de setpoints2 (ou seja 5 setpoints pois a posicao 0 ja foi definida)
			//dx/6 eh o intervalo entre cada setpoint.
			
			
			for(i = 1;i<6;i++) {
				setPointsKW23[i][0] = setPointsKW23[i-1][0] + dx/6;
				setPointsKW23[i][2] = a*setPointsKW23[i][0] + b;
			}
			
			for(i = 0; i< (int)numberOfSetPoints; i++) {
				setPointsKW01[i][0] = setPointsKW23[i][0];	setPointsKW01[i][1] = setPointsKW23[i][1];	setPointsKW01[i][2] = setPointsKW23[i][2];
			}
			
			//Muda o sinal da componente y pois para as pernas 0 e 1 o corpo esta inclinado para o outro lado
			for(i=0; i<(int) numberOfSetPoints; i++) {
				setPointsKW01[i][1] = - setPointsKW01[i][1];
			}
			

			
		}


	public void doControl()  {
			getAnklePositions();
			getCOMdata();
			
			
			k1 = 250;
			k2 = 300;
			k3 = 150;
			k4 = 300;
			kd1 = 3;
			kd2 = 5;
			kd3 = 5;
			kd4 = 3;
			
			if(initiateGait) {
				initiateGait();
			}
			
			if(initiateGait == false) {
				keepWalking(numberOfLoops);
			}
			
				
			hipToAnkleVector(posicaoPerna, flexAnkle2, abdFlexHip2);
			posicaoX.set(posicaoPerna[0]);
			posicaoY.set(posicaoPerna[1]);
			posicaoZ.set(posicaoPerna[2]);
			nloops.set(numberOfLoops);
			numberOfLoops++;
			
			rob.computeCenterOfMass(tempCOMPosition);
			COM.set(tempCOMPosition);
			ComPositionVector.setVector(COM.getVector3dCopy());
		}


	private void keepWalking(int nloops) {
		
		
		
		moveLeg();
		
		moveForward();
		
		if(finishedStep) {
			//SE TERMINOU O PASSO + MOVIMENTO HORIZONTAL
			if(pernas[0].isFlying) {
				legOffSet[0] = 0;	legOffSet[1] = 1;	legOffSet[2] = 3;	legOffSet[3] = 2;	//Cada variavel recebe o offset futuro da perna
				pernas[0].isFlying = false;
				pernas[2].isFlying = true;
				redefineLegAngles(2);
			}
			if(pernas[1].isFlying) {
				legOffSet[0] = 1;	legOffSet[1] = 2;	legOffSet[2] = 0;	legOffSet[3] = 3;	//Cada variavel recebe o offset futuro da perna	
				pernas[1].isFlying = false;
				pernas[0].isFlying = true;
				redefineLegAngles(0);
			}
			if(pernas[2].isFlying) {
				legOffSet[0] = 2;	legOffSet[1] = 3;	legOffSet[2] = 1;	legOffSet[3] = 0;	//Cada variavel recebe o offset futuro da perna
				pernas[2].isFlying = false;
				pernas[3].isFlying = true;
				redefineLegAngles(3);
			}
			if(pernas[3].isFlying) {
				legOffSet[0] = 3;	legOffSet[1] = 0;	legOffSet[2] = 2;	legOffSet[3] = 1;	//Cada variavel recebe o offset futuro da perna
				pernas[3].isFlying = false;
				pernas[1].isFlying = true;
				redefineLegAngles(1);
			}
			
			moveLegLR = false;
			finishedStep = false;
			
		}
		
	}
	


	private void moveForward() {
		if(!notMovingForward) {
			
		}
	}


	private void moveLeg() {
	//Essa funcao controla a pata que realiza o voo.
		if(notMovingForward) {
			double angles0[] = new double[3];
			double angles1[] = new double[3];
			double angles2[] = new double[3];
			double setpoints[][] = new double[(int) numberOfSetPoints][3];
			int legNum=0;
			int a[] = new int[3];
			
			//Recebe o primeiro loop
			if(!moveLegLR) {
				staticNloops = numberOfLoops;
				sp = 0;
				moveLegLR = true;
			}
			
			//Descobre qual pata realiza voo
			for(int i = 0; i< pernas.length;i++) {
				if(pernas[i].isFlying == true) {
					legNum = i;
				}
				if(legNum==1 || legNum==0) {
					setpoints = setPointsKW01.clone();
				}else {
					setpoints = setPointsKW23.clone();
				}
			}
			//A rotina SWITCH apenas permite saber quais sao as outras 3 patas que nao realizam voo o colocam-nas em um vetor a[] em ordem e com isso define os angulos de cada perna de apoio
			switch (legNum) {
				case 0:
					a[0] = 1;a[1] = 2;a[2] = 3;
					angles0[0] = staticLeg1Angles[0];	angles0[1] = staticLeg1Angles[1];	angles0[2] = staticLeg1Angles[2];
					angles1[0] = staticLeg2Angles[0];	angles1[1] = staticLeg2Angles[1];	angles1[2] = staticLeg2Angles[2];
					angles2[0] = staticLeg3Angles[0];	angles2[1] = staticLeg3Angles[1];	angles2[2] = staticLeg3Angles[2];
					break;
				case 1:
					a[0] = 0;a[1] = 2;a[2] = 3;
					angles0[0] = staticLeg0Angles[0];	angles0[1] = staticLeg0Angles[1];	angles0[2] = staticLeg0Angles[2];
					angles1[0] = staticLeg2Angles[0];	angles1[1] = staticLeg2Angles[1];	angles1[2] = staticLeg2Angles[2];
					angles2[0] = staticLeg3Angles[0];	angles2[1] = staticLeg3Angles[1];	angles2[2] = staticLeg3Angles[2];
					break;
				case 2:
					a[0] = 0;a[1] = 1;a[2] = 3;
					angles0[0] = staticLeg0Angles[0];	angles0[1] = staticLeg0Angles[1];	angles0[2] = staticLeg0Angles[2];
					angles1[0] = staticLeg1Angles[0];	angles1[1] = staticLeg1Angles[1];	angles1[2] = staticLeg1Angles[2];
					angles2[0] = staticLeg3Angles[0];	angles2[1] = staticLeg3Angles[1];	angles2[2] = staticLeg3Angles[2];
					break;
				case 3:
					a[0] = 0;a[1] = 1;a[2] = 2;
					angles0[0] = staticLeg0Angles[0];	angles0[1] = staticLeg0Angles[1];	angles0[2] = staticLeg0Angles[2];
					angles1[0] = staticLeg1Angles[0];	angles1[1] = staticLeg1Angles[1];	angles1[2] = staticLeg1Angles[2];
					angles2[0] = staticLeg2Angles[0];	angles2[1] = staticLeg2Angles[1];	angles2[2] = staticLeg2Angles[2];
					break;	
			}
			
			//Agora vamos definir qual o angulo da pata, primeiro definindo o setpoint
			if((numberOfLoops-staticNloops)%spDuration == 0) {
				sp++;
				pernas[legNum].InverseKinematics(setpoints[sp], flyingLegAngles);
				if(sp == (int)numberOfSetPoints && numberOfLoops==(numberOfSetPoints*spDuration)) {
					sp = 0;
					finishedStep = true;
					moveLegLR = false;
					notMovingForward = false;
				}
			}	
			
			disableGCP();
			
			//Controle da Junta Abdutora do Quadril
			pernas[legNum].HipJoint.getFirstJoint().setTau(k4*(flyingLegAngles[0] - pernas[legNum].HipJoint.getFirstJoint().getQ()) + kd4*(0 - pernas[legNum].HipJoint.getFirstJoint().getQD()));
			pernas[a[0]].HipJoint.getFirstJoint().setTau(k4*(angles0[0] - pernas[a[0]].HipJoint.getFirstJoint().getQ()) + kd4*(0 - pernas[a[0]].HipJoint.getQD()));
			pernas[a[1]].HipJoint.getFirstJoint().setTau(k4*(angles1[0] - pernas[a[1]].HipJoint.getFirstJoint().getQ()) + kd4*(0 - pernas[a[1]].HipJoint.getQD()));
			pernas[a[2]].HipJoint.getFirstJoint().setTau(k4*(angles2[0] - pernas[a[2]].HipJoint.getFirstJoint().getQ()) + kd4*(0 - pernas[a[2]].HipJoint.getQD()));
			
			//Controle da Junta Flexora do Quadril
			erro_qHip.set(flyingLegAngles[1] - q_flexHip2.getValueAsDouble()); erro_qdHip.set(0 - qd_flexHip2.getValueAsDouble());
			pernas[legNum].HipJoint.getSecondJoint().setTau(10*(flyingLegAngles[1] - pernas[legNum].HipJoint.getSecondJoint().getQ()) + 3*(0 - pernas[legNum].HipJoint.getSecondJoint().getQD()));	
			pernas[a[0]].HipJoint.getSecondJoint().setTau(k1*(angles0[1] - pernas[a[0]].HipJoint.getSecondJoint().getQ()) + kd1*(0 - pernas[a[0]].HipJoint.getSecondJoint().getQD()));
			pernas[a[1]].HipJoint.getSecondJoint().setTau(k1*(angles1[1] - pernas[a[1]].HipJoint.getSecondJoint().getQ()) + kd1*(0 - pernas[a[1]].HipJoint.getSecondJoint().getQD()));
			pernas[a[2]].HipJoint.getSecondJoint().setTau(k1*(angles2[1] - pernas[a[2]].HipJoint.getSecondJoint().getQ()) + kd1*(0 - pernas[a[2]].HipJoint.getSecondJoint().getQD()));
			
			//Controle da Junta Flexora do Joelho
			erro_qKnee.set(flyingLegAngles[2] - q_flexKnee2.getValueAsDouble()); erro_qdKnee.set(0 - qd_flexKnee2.getValueAsDouble());
			pernas[legNum].KneeJoint.setTau(15*(flyingLegAngles[2] - pernas[legNum].KneeJoint.getQ()) + 3*(0 - pernas[legNum].KneeJoint.getQD()));
			pernas[a[0]].KneeJoint.setTau(k2*(angles0[2] - pernas[a[0]].KneeJoint.getQ()) + kd2*(0 - pernas[a[0]].KneeJoint.getQD()));
			pernas[a[1]].KneeJoint.setTau(k2*(angles1[2] - pernas[a[1]].KneeJoint.getQ()) + kd2*(0 - pernas[a[1]].KneeJoint.getQD()));
			pernas[a[2]].KneeJoint.setTau(k2*(angles2[2] - pernas[a[2]].KneeJoint.getQ()) + kd2*(0 - pernas[a[2]].KneeJoint.getQD()));
			
			//Controle da Junta Flexora do Tornozelo
			pernas[legNum].AnkleJoint.setTau(k3*(-rob.psi - pernas[legNum].AnkleJoint.getQ()) + kd3*(0 - pernas[legNum].AnkleJoint.getQD()));
			pernas[a[0]].AnkleJoint.setTau(k3*(-rob.psi - pernas[a[0]].AnkleJoint.getQ()) + kd3*(0 - pernas[a[0]].AnkleJoint.getQD()));
			pernas[a[1]].AnkleJoint.setTau(k3*(-rob.psi - pernas[a[1]].AnkleJoint.getQ()) + kd3*(0 - pernas[a[1]].AnkleJoint.getQD()));
			pernas[a[2]].AnkleJoint.setTau(k3*(-rob.psi - pernas[a[2]].AnkleJoint.getQ()) + kd3*(0 - pernas[a[2]].AnkleJoint.getQD()));
			
		}
	}


	private void redefineLegAngles(int fleg) {
		//RECEBE A PERNA QUE REALIZARA O VOO E DEFINE OS ANGULOS DE CADA PERNA EM APOIO.
		double aux[] = new double[3];
		
		aux = staticLeg0Angles.clone();
		staticLeg0Angles = staticLeg1Angles.clone();
		staticLeg1Angles = staticLeg3Angles.clone();
		staticLeg3Angles = staticLeg2Angles.clone();
		staticLeg2Angles = aux.clone();
		//Muda o angulo da 
		if(fleg == 2 || fleg == 1) {
			staticLeg0Angles[1] = -staticLeg0Angles[1];
			staticLeg1Angles[1] = -staticLeg1Angles[1];
			staticLeg2Angles[1] = -staticLeg2Angles[1];
			staticLeg3Angles[1] = -staticLeg3Angles[1];
		}
			
	}


	private void initiateGait() {
		//ESSA FUNCAO REALIZA OS TRES PRIMEIROS PASSOS DA ANDADURA, PARA OS QUAIS OS ANGULOS DAS PERNAS EM APOIO SE ALTERAM A CADA PASSADA
		
		//INCLINACAO LATERAL DO QUADRIL
		if(numberOfLoops < leanHipDuration) {
			leanHip(-Math.PI/20);
		}
		
		//PRIMEIRO PASSO --------------------------------------------------------------------------------------------------------
		//-----------------------------------------------------------------------------------------------------------------------
		if(numberOfLoops >= leanHipDuration && numberOfLoops < ((numberOfSetPoints+1)*setPointDuration + leanHipDuration)) {
			pernas[2].isFlying = true;
			moveEndPoint(pernas, setPoints1, numberOfLoops, 1000);		
		}
		
		//MOVIMENTO HORIZONTAL DO CORPO (PRIMEIRO PASSO)
		if(numberOfLoops >= ((numberOfSetPoints+1)*setPointDuration + leanHipDuration)  && numberOfLoops <= ((numberOfSetPoints+1)*setPointDuration + leanHipDuration + forwardDuration) ) {
	
			moveBodyForward(pernas);			
			if(numberOfLoops == ((numberOfSetPoints+1)*setPointDuration + leanHipDuration + forwardDuration)) {
				//Se finalizou o movimento horizontal o voo acabou entao a variavel boolean recebe false e zera-se o contador de setpoints para o proximo voo
				pernas[2].isFlying = false;
				currentSetPoint = 0;
			}
			
		}
		
		//SEGUNDO PASSO  --------------------------------------------------------------------------------------------------------
		//-----------------------------------------------------------------------------------------------------------------------
		if(numberOfLoops > ((numberOfSetPoints+1)*setPointDuration + leanHipDuration + forwardDuration) && numberOfLoops <= (2*(numberOfSetPoints+1)*setPointDuration + leanHipDuration + forwardDuration)) {
			pernas[3].isFlying = true;
			moveEndPoint(pernas, setPoints2, numberOfLoops, 1000);
		}
		
		if(numberOfLoops > (2*(numberOfSetPoints+1)*setPointDuration + leanHipDuration + forwardDuration)	&&	numberOfLoops <= (2*(numberOfSetPoints+1)*setPointDuration + leanHipDuration + 2*forwardDuration)) {
			moveBodyForward(pernas);
			if(numberOfLoops == (2*(numberOfSetPoints+1)*setPointDuration + leanHipDuration + 2*forwardDuration)) {
				//Se finalizou o movimento horizontal o voo acabou entao a variavel boolean recebe false e zera-se o contador de setpoints para o proximo voo
				pernas[3].isFlying = false;
				currentSetPoint = 0;
			}
		}
		
		if(numberOfLoops > (2*(numberOfSetPoints+1)*setPointDuration + leanHipDuration + 2*forwardDuration) && numberOfLoops <= (2*(numberOfSetPoints+1)*setPointDuration + 4*leanHipDuration + 2*forwardDuration)) {
			leanHip(0.0);
		}
		if(numberOfLoops > (2*(numberOfSetPoints+1)*setPointDuration + 4*leanHipDuration + 2*forwardDuration ) && numberOfLoops <=(2*(numberOfSetPoints+1)*setPointDuration + 6*leanHipDuration + 2*forwardDuration )) {
			leanHip(Math.PI/20);
		}
		
		//TERCEIRO PASSO --------------------------------------------------------------------------------------------------------
		//-----------------------------------------------------------------------------------------------------------------------
		if(numberOfLoops > (2*(numberOfSetPoints+1)*setPointDuration + 6*leanHipDuration + 2*forwardDuration )	&&	numberOfLoops <= (3*(numberOfSetPoints+1)*setPointDuration + 6*leanHipDuration + 2*forwardDuration)) {
			pernas[1].isFlying = true;
			moveEndPoint(pernas,setPoints3,numberOfLoops, 1000);
		}
		if(numberOfLoops > (3*(numberOfSetPoints+1)*setPointDuration + 6*leanHipDuration + 2*forwardDuration)	&&	numberOfLoops <= (3*(numberOfSetPoints+1)*setPointDuration + 6*leanHipDuration + 3*forwardDuration) ) {
			moveBodyForward(pernas);
			if(numberOfLoops == (3*(numberOfSetPoints+1)*setPointDuration + 6*leanHipDuration + 3*forwardDuration) ) {
				pernas[1].isFlying = false;
				currentSetPoint = 0;
				initiateGait = false; //AO CHEGAR NESSE PONTO A FUNCAO INITIATEGAIT() PARA DE SER CHAMADA
				pernas[0].isFlying = true;
			}
		}
	}


	private void moveBodyForward(GuaraLeg []legs) {
		
		if(legs[0].isFlying == true) {
			//step4Forward(pernas); //Quarto Passo
		}
		if(legs[1].isFlying == true) {
			step3Forward(pernas);	//Terceiro Passo 
		}
		if(legs[2].isFlying == true) {
			step1Forward(pernas);	//Primeiro passo
			
		}
		if(legs[3].isFlying == true) {
			step2Forward(pernas);	//Segundo Passo
		}
		
		//APOS REALIZADO O VOO DA PATA SERA FEITO O MOVIMENTO HORIZONTAL DO CORPO.
		if(calculouFA == false) {
			//CALCULANDO OS ANGULOS DE JUNTA FINAIS
			hipToAnkleVector(forwardVector, flexAnkle1, abdFlexHip1);
			forwardVector[0] = -stepSize;
			inverseKinematics(forwardAngles, forwardVector);
			calculouFA = true;
			System.out.println("-------------- [" + forwardVector[0] + ", " + forwardVector[1] + ", " + forwardVector[2] + "]" + " ------------------------");
			System.out.println("-------------- [" + forwardAngles[0] + ", " + forwardAngles[1] + ", " + forwardAngles[2] + "]" + " ------------------------");
			//-0.18379098411121483, 0.8973165008933564, -1.0125607640721184
			//-0.18379098411121483, 0.8693134643277585, -0.6309487034248928
		}
		
	}

	private void step1Forward(GuaraLeg []legs) {
		//A perna 2 anda 1 stepSize para frente e vai para a posicao origial. As demais pernas se inclinam sp[-1*stepsize,y,z]
		double angulosPerna2[] = {-0.15707963267948966/*-0.18379098411121483*/, 0.5856855434571511, -1.1713710869143021};
		double angulosOutrasPernas[] = {-0.15707963267948966/*-0.18379098411121483*/, 0.8029286145136825, -1.1992853145260651};
		
		legs[2].HipJoint.getFirstJoint().setTau( 300*(angulosPerna2[0] - legs[2].HipJoint.getFirstJoint().getQ()) + 3*(0 - legs[2].HipJoint.getFirstJoint().getQD()) );
		legs[0].HipJoint.getFirstJoint().setTau( 300*(angulosOutrasPernas[0]/*-(Math.PI/20)*/ - q_abdHip0.getValueAsDouble()) + 3*(0 - qd_abdHip0.getValueAsDouble()) );
		legs[1].HipJoint.getFirstJoint().setTau( 300*(angulosOutrasPernas[0]/*-(Math.PI/20)*/ - q_abdHip0.getValueAsDouble()) + 3*(0 - qd_abdHip0.getValueAsDouble()) );
		legs[3].HipJoint.getFirstJoint().setTau( 300*(angulosOutrasPernas[0]/*-(Math.PI/20)*/ - q_abdHip0.getValueAsDouble()) + 3*(0 - qd_abdHip0.getValueAsDouble()) );
		
		legs[2].HipJoint.getSecondJoint().setTau( 150*(angulosPerna2[1] - legs[2].HipJoint.getSecondJoint().getQ()) + 3*(0 - legs[2].HipJoint.getSecondJoint().getQD()) );
		legs[0].HipJoint.getSecondJoint().setTau( 150*(angulosOutrasPernas[1] - legs[0].HipJoint.getSecondJoint().getQ()) + 5*(0 - legs[0].HipJoint.getSecondJoint().getQD()) );
		legs[1].HipJoint.getSecondJoint().setTau( 150*(angulosOutrasPernas[1] - legs[1].HipJoint.getSecondJoint().getQ()) + 5*(0 - legs[1].HipJoint.getSecondJoint().getQD()) );
		legs[3].HipJoint.getSecondJoint().setTau( 150*(angulosOutrasPernas[1] - legs[3].HipJoint.getSecondJoint().getQ()) + 5*(0 - legs[3].HipJoint.getSecondJoint().getQD()) );
		
		legs[2].KneeJoint.setTau( 100*(angulosPerna2[2] - legs[2].KneeJoint.getQ()) + 3*(0 - legs[2].KneeJoint.getQD()) );
		legs[0].KneeJoint.setTau( 150*(angulosOutrasPernas[2] - legs[0].KneeJoint.getQ()) + 3*(0 - legs[0].KneeJoint.getQD()) );
		legs[1].KneeJoint.setTau( 150*(angulosOutrasPernas[2] - legs[1].KneeJoint.getQ()) + 3*(0 - legs[1].KneeJoint.getQD()) );
		legs[3].KneeJoint.setTau( 150*(angulosOutrasPernas[2] - legs[3].KneeJoint.getQ()) + 3*(0 - legs[3].KneeJoint.getQD()) );
		
		legs[2].AnkleJoint.setTau( 150*(-rob.psi - legs[2].AnkleJoint.getQ()) + 5*(0 - legs[2].AnkleJoint.getQD()) );
		legs[0].AnkleJoint.setTau( 150*(-rob.psi - legs[0].AnkleJoint.getQ()) + 5*(0 - legs[0].AnkleJoint.getQD()) );
		legs[1].AnkleJoint.setTau( 150*(-rob.psi - legs[1].AnkleJoint.getQ()) + 5*(0 - legs[1].AnkleJoint.getQD()) );
		legs[3].AnkleJoint.setTau( 150*(-rob.psi - legs[3].AnkleJoint.getQ()) + 5*(0 - legs[3].AnkleJoint.getQD()) );
		
		//Ao interromper o controle que movimenta o corpo para frente, ainda serao necessarios saber os angulos de junta das pernas que permanecerao no chao no proximo passo
		//por isso, as variaveis a seguir servirao para isso, passando os valores dos angulos para elas:
		staticLeg0Angles[0] = angulosOutrasPernas[0]; 		staticLeg0Angles[1] = angulosOutrasPernas[1];		staticLeg0Angles[2] = angulosOutrasPernas[2];
		staticLeg1Angles[0] = angulosOutrasPernas[0]; 		staticLeg1Angles[1] = angulosOutrasPernas[1];		staticLeg1Angles[2] = angulosOutrasPernas[2];
		staticLeg2Angles[0] = angulosPerna2[0];				staticLeg2Angles[1] = angulosPerna2[1];				staticLeg2Angles[2] = angulosPerna2[2];
		staticLeg3Angles[0] = angulosOutrasPernas[0]; 		staticLeg3Angles[1] = angulosOutrasPernas[1];		staticLeg3Angles[2] = angulosOutrasPernas[2];
	}
	
	private void step2Forward(GuaraLeg []legs) {
		//A perna 3 anda 1 stepSize para frente  e vai para a posicao origial. A perna 2 inclina de sp[-1*stepSize, y, z]
		//As demais pernas mudam a inclinacao para sp[-2*stepSize, y, z]
		
		double angulosPerna3[] = { -0.15707963267948966/*-0.18379098411121483*/, 0.5856855434571511, -1.1713710869143021 };
		double angulosPerna2[] = {-0.15707963267948966/* -0.18379098411121483*/, 0.8029286145136825, -1.1992853145260651 };
		double angulosOutrasPernas[] = { -0.15707963267948966/*-0.18379098411121483*/, 0.8973165008933564, -1.0125607640721184 }; //Angulos de 2 steps para tras
		
		legs[2].HipJoint.getFirstJoint().setTau( 300*(angulosPerna2[0] - legs[2].HipJoint.getFirstJoint().getQ()) + 3*(0 - legs[2].HipJoint.getFirstJoint().getQD()) );
		legs[0].HipJoint.getFirstJoint().setTau( 300*(-(Math.PI/20) - legs[0].HipJoint.getFirstJoint().getQ()) + 3*(0 - qd_abdHip0.getValueAsDouble()) );
		legs[1].HipJoint.getFirstJoint().setTau( 300*(-(Math.PI/20) - legs[1].HipJoint.getFirstJoint().getQ()) + 3*(0 - qd_abdHip0.getValueAsDouble()) );
		legs[3].HipJoint.getFirstJoint().setTau( 300*(angulosPerna3[0] - legs[3].HipJoint.getFirstJoint().getQ()) + 3*(0 - qd_abdHip0.getValueAsDouble()) );
		
		legs[2].HipJoint.getSecondJoint().setTau( 150*(angulosPerna2[1] - legs[2].HipJoint.getSecondJoint().getQ()) + 3*(0 - legs[2].HipJoint.getSecondJoint().getQD()) );
		legs[0].HipJoint.getSecondJoint().setTau( 150*(angulosOutrasPernas[1] - legs[0].HipJoint.getSecondJoint().getQ()) + 5*(0 - legs[0].HipJoint.getSecondJoint().getQD()) );
		legs[1].HipJoint.getSecondJoint().setTau( 150*(angulosOutrasPernas[1] - legs[1].HipJoint.getSecondJoint().getQ()) + 5*(0 - legs[1].HipJoint.getSecondJoint().getQD()) );
		legs[3].HipJoint.getSecondJoint().setTau( 150*(angulosPerna3[1] - legs[3].HipJoint.getSecondJoint().getQ()) + 5*(0 - legs[3].HipJoint.getSecondJoint().getQD()) );
		
		legs[2].KneeJoint.setTau( 100*(angulosPerna2[2] - legs[2].KneeJoint.getQ()) + 3*(0 - legs[2].KneeJoint.getQD()) );
		legs[0].KneeJoint.setTau( 150*(angulosOutrasPernas[2] - legs[0].KneeJoint.getQ()) + 3*(0 - legs[0].KneeJoint.getQD()) );
		legs[1].KneeJoint.setTau( 150*(angulosOutrasPernas[2] - legs[1].KneeJoint.getQ()) + 3*(0 - legs[1].KneeJoint.getQD()) );
		legs[3].KneeJoint.setTau( 150*(angulosPerna3[2] - legs[3].KneeJoint.getQ()) + 3*(0 - legs[3].KneeJoint.getQD()) );
		
		legs[2].AnkleJoint.setTau( 150*(-rob.psi - legs[2].AnkleJoint.getQ()) + 5*(0 - legs[2].AnkleJoint.getQD()) );
		legs[0].AnkleJoint.setTau( 150*(-rob.psi - legs[0].AnkleJoint.getQ()) + 5*(0 - legs[0].AnkleJoint.getQD()) );
		legs[1].AnkleJoint.setTau( 150*(-rob.psi - legs[1].AnkleJoint.getQ()) + 5*(0 - legs[1].AnkleJoint.getQD()) );
		legs[3].AnkleJoint.setTau( 150*(-rob.psi - legs[3].AnkleJoint.getQ()) + 5*(0 - legs[3].AnkleJoint.getQD()) );
		
		//Ao interromper o controle que movimenta o corpo para frente, ainda serao necessarios saber os angulos de junta das pernas que permanecerao no chao no proximo passo
		//por isso, as variaveis a seguir servirao para isso, passando os valores dos angulos para elas:
		staticLeg0Angles[0] = angulosOutrasPernas[0]; 		staticLeg0Angles[1] = angulosOutrasPernas[1];		staticLeg0Angles[2] = angulosOutrasPernas[2];
		staticLeg1Angles[0] = angulosOutrasPernas[0]; 		staticLeg1Angles[1] = angulosOutrasPernas[1];		staticLeg1Angles[2] = angulosOutrasPernas[2];
		staticLeg2Angles[0] = angulosPerna2[0];				staticLeg2Angles[1] = angulosPerna2[1];				staticLeg2Angles[2] = angulosPerna2[2];
		staticLeg3Angles[0] = angulosPerna3[0]; 			staticLeg3Angles[1] = angulosPerna3[1];				staticLeg3Angles[2] = angulosPerna3[2];
		
	}
	
	private void step3Forward(GuaraLeg []legs) {
		//A perna 1 anda 1 stepsize para tras e vai para a posicao origial. A perna 2 muda a inclinacao para 2 steps. A perna 3 inclina para 1 step para tras
		//A perna 0 muda a inclinacao para sp[-3*stepSize, y, z], atingindo seu limite cinematico.
		
		double angulosPerna1[] = {0.15707963267948966/*-0.18379098411121483*/, 0.5856855434571511, -1.1713710869143021};
		double angulosPerna3[] = {0.15707963267948966/*-0.18379098411121483*/, 0.8029286145136825, -1.1992853145260651};
		double angulosPerna2[] = {0.15707963267948966/*-0.18379098411121483*/, 0.8973165008933564, -1.0125607640721184};
		double angulosPerna0[] = {0.15707963267948966/*-0.18379098411121483*/, 0.8693134643277585, -0.6309487034248928};
		
		legs[2].HipJoint.getFirstJoint().setTau( 300*(angulosPerna2[0] - legs[2].HipJoint.getFirstJoint().getQ()) + 3*(0 - legs[2].HipJoint.getFirstJoint().getQD()) );
		legs[0].HipJoint.getFirstJoint().setTau( 300*(angulosPerna0[0] - legs[0].HipJoint.getFirstJoint().getQ()) + 3*(0 - legs[0].HipJoint.getFirstJoint().getQD()) );
		legs[1].HipJoint.getFirstJoint().setTau( 300*(angulosPerna1[0] - legs[1].HipJoint.getFirstJoint().getQ()) + 3*(0 - legs[1].HipJoint.getFirstJoint().getQD()) );
		legs[3].HipJoint.getFirstJoint().setTau( 300*(angulosPerna3[0] - legs[3].HipJoint.getFirstJoint().getQ()) + 3*(0 - legs[3].HipJoint.getFirstJoint().getQD()) );
		
		legs[2].HipJoint.getSecondJoint().setTau( 150*(angulosPerna2[1] - legs[2].HipJoint.getSecondJoint().getQ()) + 3*(0 - legs[2].HipJoint.getSecondJoint().getQD()) );
		legs[0].HipJoint.getSecondJoint().setTau( 150*(angulosPerna0[1] - legs[0].HipJoint.getSecondJoint().getQ()) + 5*(0 - legs[0].HipJoint.getSecondJoint().getQD()) );
		legs[1].HipJoint.getSecondJoint().setTau( 150*(angulosPerna1[1] - legs[1].HipJoint.getSecondJoint().getQ()) + 5*(0 - legs[1].HipJoint.getSecondJoint().getQD()) );
		legs[3].HipJoint.getSecondJoint().setTau( 150*(angulosPerna3[1] - legs[3].HipJoint.getSecondJoint().getQ()) + 5*(0 - legs[3].HipJoint.getSecondJoint().getQD()) );
		
		legs[2].KneeJoint.setTau( 100*(angulosPerna2[2] - legs[2].KneeJoint.getQ()) + 3*(0 - legs[2].KneeJoint.getQD()) );
		legs[0].KneeJoint.setTau( 150*(angulosPerna0[2] - legs[0].KneeJoint.getQ()) + 3*(0 - legs[0].KneeJoint.getQD()) );
		legs[1].KneeJoint.setTau( 150*(angulosPerna1[2] - legs[1].KneeJoint.getQ()) + 3*(0 - legs[1].KneeJoint.getQD()) );
		legs[3].KneeJoint.setTau( 150*(angulosPerna3[2] - legs[3].KneeJoint.getQ()) + 3*(0 - legs[3].KneeJoint.getQD()) );
		
		legs[2].AnkleJoint.setTau( 150*(-rob.psi - legs[2].AnkleJoint.getQ()) + 5*(0 - legs[2].AnkleJoint.getQD()) );
		legs[0].AnkleJoint.setTau( 150*(-rob.psi - legs[0].AnkleJoint.getQ()) + 5*(0 - legs[0].AnkleJoint.getQD()) );
		legs[1].AnkleJoint.setTau( 150*(-rob.psi - legs[1].AnkleJoint.getQ()) + 5*(0 - legs[1].AnkleJoint.getQD()) );
		legs[3].AnkleJoint.setTau( 150*(-rob.psi - legs[3].AnkleJoint.getQ()) + 5*(0 - legs[3].AnkleJoint.getQD()) );
		
		//Ao interromper o controle que movimenta o corpo para frente, ainda serao necessarios saber os angulos de junta das pernas que permanecerao no chao no proximo passo
		//por isso, as variaveis a seguir servirao para isso, passando os valores dos angulos para elas:
		staticLeg0Angles[0] = angulosPerna0[0]; 		staticLeg0Angles[1] = angulosPerna0[1];		staticLeg0Angles[2] = angulosPerna0[2];
		staticLeg1Angles[0] = angulosPerna1[0]; 		staticLeg1Angles[1] = angulosPerna1[1];		staticLeg1Angles[2] = angulosPerna1[2];
		staticLeg2Angles[0] = angulosPerna2[0];			staticLeg2Angles[1] = angulosPerna2[1];		staticLeg2Angles[2] = angulosPerna2[2];
		staticLeg3Angles[0] = angulosPerna3[0]; 		staticLeg3Angles[1] = angulosPerna3[1];		staticLeg3Angles[2] = angulosPerna3[2];
		
	}
	

	
	private void moveEndPoint(GuaraLeg []legs, double [][]sps, int nloops, int spDuration) {
		//A rotina if a seguir guarda o valor da primeira entrada do numero de setpoints, para saber quando calcular os angulos de cinematica inversa;
		if(nloopsReceived == false && currentSetPoint == 0) {
			firstNLoops = nloops;
			nloopsReceived = true;
		}
		
		disableGCP(); //Desabilita o contato do GCP (apenas) para a perna que realiza o voo	
			
		//A rotina if a seguir recalcula os angulos de junta para cada setPoint
		if((firstNLoops - nloops)%spDuration == 0 && currentSetPoint < numberOfSetPoints) {
			System.out.println("Entrou na Cinematica Inversa - Loop: " + nloops);
			
			inverseKinematics(flyingLegAngles, sps[currentSetPoint]);
			currentSetPoint++;
			currentSP.set(currentSetPoint); //Passando valor para YoDouble
			
			System.out.print("SetPoint " + currentSetPoint + " : " + "[" + flyingLegAngles[0] + ", " + flyingLegAngles[1] + ", " + flyingLegAngles[2] + "]");
			System.out.println(" ");
			System.out.println("Saiu da Cinematica Inversa");
			
			legAngle1.set(flyingLegAngles[1]);//Passando valor para YoDouble
			legAngle2.set(flyingLegAngles[2]);//Passando valor para YoDouble
			
			//A rotina if  a seguir define a variavel booleana como falsa para que a proxima chamada da funcao utilize-a;
			if(currentSetPoint == numberOfSetPoints) {
				nloopsReceived = false;
			}
		}
		
		//Os if's a seguir realizam o controle da perna, verificando qual perna deve realizar o voo por meio da variavel booleana de cada pata 'isFlying'
		if(legs[0].isFlying == true) {
			flyLeg(0, legs);
		}
		
		if(legs[1].isFlying == true) {
			flyLeg(1, legs);
		}
		
		if(legs[2].isFlying == true) {
			flyLeg(2, legs);
		}
		
		if(legs[3].isFlying == true) {
			flyLeg(3, legs);
		}
		
	}
	


	
	private void disableGCP() {
		
		if(pernas[0].isFlying == true) {
			pernas[0].gcHeel.setNotInContact();
			gcp0.set(false);
		}else {		gcp0.set(true);		}
		
		if(pernas[1].isFlying == true) {
			pernas[1].gcHeel.setNotInContact();
			gcp1.set(false);
		}else {		gcp1.set(true);		}
		
		if(pernas[2].isFlying == true) {
			pernas[2].gcHeel.setNotInContact();
			gcp2.set(false);
		}else {		gcp2.set(true);		}
		
		if(pernas[3].isFlying == true) {
			pernas[3].gcHeel.setNotInContact();
			gcp3.set(false);
		}else {		gcp3.set(true);		}
		
	}

	public void flyLeg(int legNum, GuaraLeg []pernas) {
		//Essa funcao controla a pata que realiza o voo, ela recebe o numero da pata que realiza o voo e o vetor com as quatro patas.
		//A rotina SWITCH apenas permite saber quais sao as outras 3 patas que nao realizam voo
		double angles0[] = new double[3];
		double angles1[] = new double[3];
		double angles2[] = new double[3];
		
		int a[] = new int[3];
		switch (legNum) {
			case 0:
				a[0] = 1;a[1] = 2;a[2] = 3;
				angles0[0] = staticLeg1Angles[0];	angles0[1] = staticLeg1Angles[1];	angles0[2] = staticLeg1Angles[2];
				angles1[0] = staticLeg2Angles[0];	angles1[1] = staticLeg2Angles[1];	angles1[2] = staticLeg2Angles[2];
				angles2[0] = staticLeg3Angles[0];	angles2[1] = staticLeg3Angles[1];	angles2[2] = staticLeg3Angles[2];
				break;
			case 1:
				a[0] = 0;a[1] = 2;a[2] = 3;
				angles0[0] = staticLeg0Angles[0];	angles0[1] = staticLeg0Angles[1];	angles0[2] = staticLeg0Angles[2];
				angles1[0] = staticLeg2Angles[0];	angles1[1] = staticLeg2Angles[1];	angles1[2] = staticLeg2Angles[2];
				angles2[0] = staticLeg3Angles[0];	angles2[1] = staticLeg3Angles[1];	angles2[2] = staticLeg3Angles[2];
				break;
			case 2:
				a[0] = 0;a[1] = 1;a[2] = 3;
				angles0[0] = staticLeg0Angles[0];	angles0[1] = staticLeg0Angles[1];	angles0[2] = staticLeg0Angles[2];
				angles1[0] = staticLeg1Angles[0];	angles1[1] = staticLeg1Angles[1];	angles1[2] = staticLeg1Angles[2];
				angles2[0] = staticLeg3Angles[0];	angles2[1] = staticLeg3Angles[1];	angles2[2] = staticLeg3Angles[2];
				break;
			case 3:
				a[0] = 0;a[1] = 1;a[2] = 2;
				angles0[0] = staticLeg0Angles[0];	angles0[1] = staticLeg0Angles[1];	angles0[2] = staticLeg0Angles[2];
				angles1[0] = staticLeg1Angles[0];	angles1[1] = staticLeg1Angles[1];	angles1[2] = staticLeg1Angles[2];
				angles2[0] = staticLeg2Angles[0];	angles2[1] = staticLeg2Angles[1];	angles2[2] = staticLeg2Angles[2];
				break;
				
		}		
		//Controle da Junta Abdutora do Quadril
		pernas[legNum].HipJoint.getFirstJoint().setTau(k4*(flyingLegAngles[0] - pernas[legNum].HipJoint.getFirstJoint().getQ()) + kd4*(0 - pernas[legNum].HipJoint.getFirstJoint().getQD()));
		pernas[a[0]].HipJoint.getFirstJoint().setTau(k4*(/*-(Math.PI/20)*/angles0[0] - pernas[a[0]].HipJoint.getFirstJoint().getQ()) + kd4*(0 - pernas[a[0]].HipJoint.getQD()));
		pernas[a[1]].HipJoint.getFirstJoint().setTau(k4*(/*-(Math.PI/20)*/angles1[0] - pernas[a[1]].HipJoint.getFirstJoint().getQ()) + kd4*(0 - pernas[a[1]].HipJoint.getQD()));
		pernas[a[2]].HipJoint.getFirstJoint().setTau(k4*(/*-(Math.PI/20)*/angles2[0] - pernas[a[2]].HipJoint.getFirstJoint().getQ()) + kd4*(0 - pernas[a[2]].HipJoint.getQD()));
		
		//Controle da Junta Flexora do Quadril
		erro_qHip.set(flyingLegAngles[1] - q_flexHip2.getValueAsDouble()); erro_qdHip.set(0 - qd_flexHip2.getValueAsDouble());
		pernas[legNum].HipJoint.getSecondJoint().setTau(10*(flyingLegAngles[1] - pernas[legNum].HipJoint.getSecondJoint().getQ()) + 3*(0 - pernas[legNum].HipJoint.getSecondJoint().getQD()));	
		pernas[a[0]].HipJoint.getSecondJoint().setTau(k1*(/*rob.phiY*/angles0[1] - pernas[a[0]].HipJoint.getSecondJoint().getQ()) + kd1*(0 - pernas[a[0]].HipJoint.getSecondJoint().getQD()));
		pernas[a[1]].HipJoint.getSecondJoint().setTau(k1*(/*rob.phiY*/angles1[1] - pernas[a[1]].HipJoint.getSecondJoint().getQ()) + kd1*(0 - pernas[a[1]].HipJoint.getSecondJoint().getQD()));
		pernas[a[2]].HipJoint.getSecondJoint().setTau(k1*(/*rob.phiY*/angles2[1] - pernas[a[2]].HipJoint.getSecondJoint().getQ()) + kd1*(0 - pernas[a[2]].HipJoint.getSecondJoint().getQD()));
		
		//Controle da Junta Flexora do Joelho
		erro_qKnee.set(flyingLegAngles[2] - q_flexKnee2.getValueAsDouble()); erro_qdKnee.set(0 - qd_flexKnee2.getValueAsDouble());
		pernas[legNum].KneeJoint.setTau(15*(flyingLegAngles[2] - pernas[legNum].KneeJoint.getQ()) + 3*(0 - pernas[legNum].KneeJoint.getQD()));
		pernas[a[0]].KneeJoint.setTau(k2*(/*-rob.theta*/angles0[2] - pernas[a[0]].KneeJoint.getQ()) + kd2*(0 - pernas[a[0]].KneeJoint.getQD()));
		pernas[a[1]].KneeJoint.setTau(k2*(/*-rob.theta*/angles1[2] - pernas[a[1]].KneeJoint.getQ()) + kd2*(0 - pernas[a[1]].KneeJoint.getQD()));
		pernas[a[2]].KneeJoint.setTau(k2*(/*-rob.theta*/angles2[2] - pernas[a[2]].KneeJoint.getQ()) + kd2*(0 - pernas[a[2]].KneeJoint.getQD()));
		
		//Controle da Junta Flexora do Tornozelo
		pernas[legNum].AnkleJoint.setTau(k3*(-rob.psi - pernas[legNum].AnkleJoint.getQ()) + kd3*(0 - pernas[legNum].AnkleJoint.getQD()));
		pernas[a[0]].AnkleJoint.setTau(k3*(-rob.psi - pernas[a[0]].AnkleJoint.getQ()) + kd3*(0 - pernas[a[0]].AnkleJoint.getQD()));
		pernas[a[1]].AnkleJoint.setTau(k3*(-rob.psi - pernas[a[1]].AnkleJoint.getQ()) + kd3*(0 - pernas[a[1]].AnkleJoint.getQD()));
		pernas[a[2]].AnkleJoint.setTau(k3*(-rob.psi - pernas[a[2]].AnkleJoint.getQ()) + kd3*(0 - pernas[a[2]].AnkleJoint.getQD()));
		

	}


   public String getDescription()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public String getName()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      // TODO Auto-generated method stub
      return registry;
   }

   public void initialize()
   {
      // TODO Auto-generated method stub

   }
   
   
   private double defineFunctionAngle(double stepSize) {
	   double angle, functionLenght, stepHeight;
	   
	   stepHeight = stepSize*Math.sqrt(3);
	   
	   angle = Math.asin(stepSize/stepHeight);
	   return angle;
   }
   
   
   private void inverseKinematics(double []theta, double []vector) {
	  //O vetor theta eh o vetor que carregara os angulos de junta
	  //O vetor vector eh o vetor posicao do end efector (ankleJoint) a partir do abdHip
	   
	   double l2 = Math.pow(vector[0], 2) + Math.pow(vector[1], 2) + Math.pow(vector[2], 2);
	   
	   theta[0] =- Math.atan(vector[1]/vector[2]);
	   
	   theta[2] = Math.acos( ( l2 - Math.pow(rob.lThighZ, 2) - Math.pow(rob.lShankZ, 2) ) / (2*rob.lThighZ*rob.lShankZ) );
	   
	   theta[1] = Math.asin(rob.lShankZ*Math.sin(theta[2])/(Math.sqrt(l2))) - Math.atan(vector[0]/Math.sqrt(Math.pow(vector[1], 2)+Math.pow(vector[2], 2)));
	   
	   theta[2] = - theta[2];
	   
   }
   
	private void hipToAnkleVector(double []auxsp, PinJoint flexAnkle, UniversalJoint abdFlexHip) {
		//ESTA FUNCAO RECEBE A JUNTA DO TORNOZELO E A JUNTA DO QUADRIL E RETORNA O VETOR (QUADRIL--->TORNOZELO)
		//POR MEIO DO VETOR *DOUBLE* AUXSP(x,y,z)
		//ISSO EH FEITO POR MEIO DA SUBTRACAO DOS VETORES POSICAO DAS JUNTAS EM RELACAO AO MUNDO
		FrameVector3D worldToAnkle = new FrameVector3D();
		FrameVector3D worldToHip = new FrameVector3D();
		FrameVector3D hipToAnkle = new FrameVector3D();
		
		flexAnkle.getTranslationToWorld(worldToAnkle.getVector());
		abdFlexHip.getTranslationToWorld(worldToHip.getVector());
		hipToAnkle.sub(worldToAnkle, worldToHip);
		
		auxsp[0] = hipToAnkle.getX();
		auxsp[1] = hipToAnkle.getY();
		auxsp[2] = hipToAnkle.getZ();
		
	}
	

	
	private void defineSetPoints1(double [][]setPoints,  double []hipToAnkle, double stepSize, double numSPs) {
		//ESSE METODO CALCULA TODOS OS SETPOINTS DA TRAJETORIA HEXAGONAL DA PATA
		//O NUMERO DE SETPOINTS EH DEFINIDO PELO USUARIO
		//RECEBE-SE O A LISTA DE VETORES QUE GUARDA OS SETPOINTS, O NUMERO DE SETPOINTS, O VETOR QUADRIL---->TORNOZELO
		//E O TAMANHO DO PASSO, EQUIVALENTE AO LADO DO HEXAGONO
		//         ._._._._.
		//        ./	   \.    
		//		 ./		    \.
		//	   	./			 \.
		//		.\			 /.
		//		 .\		    /.
		//		  .\       /.
		//
		
		double alfa = defineFunctionAngle(stepSize); // Coeficiente Angular da funcao que gera os SetPoints.
		double functionInits[][] = new double[6][3]; // Lista de vetores que guarda os pontos de mudanca da funcao que gera os sp`s.
		int currentFunction, count, count2;			//Variaveis auxiliares de contadores
		double setPointsError = 0.000001;			//Erro admissivel entre setpoint e functionInit
		
		//Calculando os pontos de mudanca de funcao de funcao
		functionInits[0] = hipToAnkle;
		
		functionInits[1][0] = hipToAnkle[0] - stepSize*Math.acos(alfa);
		functionInits[1][1] = hipToAnkle[1];
		functionInits[1][2] = hipToAnkle[2] + stepSize*Math.asin(alfa);
		
		functionInits[2][0] = functionInits[1][0] + stepSize*Math.acos(alfa);
		functionInits[2][1] = hipToAnkle[1];
		functionInits[2][2] = functionInits[1][2] + stepSize*Math.asin(alfa);
		
		functionInits[3][0] = functionInits[2][0] + stepSize;
		functionInits[3][1] = hipToAnkle[1];
		functionInits[3][2] = functionInits[2][2];
		
		functionInits[4][0] = functionInits[3][0] + stepSize*Math.acos(alfa);
		functionInits[4][1] = hipToAnkle[1];
		functionInits[4][2] = functionInits[3][2] - stepSize*Math.asin(alfa);
		
		functionInits[5][0] = functionInits[4][0] - stepSize*Math.acos(alfa);
		functionInits[5][1] = hipToAnkle[1];
		functionInits[5][2] = functionInits[4][2] - stepSize*Math.asin(alfa);
		
		
		//Vamos calcular e definir a posicao de todos os setpoints
		//sendo que o primeiro setpoint eh necessariamente a posicao inicial da perna
		//que nesse momento esta sendo guardada pelo auxSetpoint
		for(count=0;count<3;count++) {
				setPoints[0][count] = hipToAnkle[count];
		}
		
		currentFunction = 0;
		
		for(count2 = 1; count2 < (int)numSPs; count2++) {
			setPoints[count2][1] = hipToAnkle[1];
			
			switch(currentFunction) {
				case(0):
					setPoints[count2][0] = setPoints[count2-1][0] - stepSize*Math.acos(alfa)/((numSPs-1)/5);
					setPoints[count2][1] = hipToAnkle[1];
					setPoints[count2][2] = setPoints[count2-1][2] + stepSize*Math.asin(alfa)/((numSPs-1)/5);
					break;
				case(1):
					setPoints[count2][0] = setPoints[count2-1][0] + stepSize*Math.acos(alfa)/((numSPs-1)/5);
					setPoints[count2][1] = hipToAnkle[1];	
					setPoints[count2][2] = setPoints[count2-1][2] + stepSize*Math.asin(alfa)/((numSPs-1)/5);
					break;
				case(2):
					setPoints[count2][0] =  setPoints[count2-1][0] + stepSize/((numSPs-1)/5);
					setPoints[count2][1] = hipToAnkle[1];
					setPoints[count2][2] = setPoints[count2-1][2];
					break;
				case(3):
					setPoints[count2][0] = setPoints[count2-1][0] + stepSize*Math.acos(alfa)/((numSPs-1)/5); 
					setPoints[count2][1] = hipToAnkle[1];
					setPoints[count2][2] = setPoints[count2-1][2] - stepSize*Math.asin(alfa)/((numSPs-1)/5);
					break;
				case(4):
					setPoints[count2][0] = setPoints[count2-1][0] - stepSize*Math.acos(alfa)/((numSPs-1)/5); 
					setPoints[count2][1] = hipToAnkle[1];
					setPoints[count2][2] = setPoints[count2-1][2] - stepSize*Math.asin(alfa)/((numSPs-1)/5);
					break;
			}
			
			
			//Controla qual funcao eh utilizada para calcular os setpoints
			//ou seja, controlao switch/case
			if(((Math.abs(setPoints[count2][0] - functionInits[currentFunction + 1][0]) <= setPointsError)) && (Math.abs(setPoints[count2][2] - functionInits[currentFunction + 1][2]) <= setPointsError)){
				
				currentFunction++;
	
			}
			
		}
		
		for(i=0;i<numSPs;i++) {
			System.out.println("[" + setPoints[i][0] + ", " + setPoints[i][1] + ", " + setPoints[i][2] + "]");
		}
		
	}
	
	private void defineSetPoints2(double[][] setPoints2, double[][] setPoints1) {
		// Vamos partir do setPoint 7 dos setPoints 1, e a partir dele calcular todos os setPoints anteriores.
		//Sabemos que o setPoint final da perna 3 eh [-0.05, -0.04432383389743616, -0.23847004473511105]
		//Variaveis para interpolacao linar: z = a*x + b
		double b, a, dx, dz;
		//Passando valores do setpoint1 para o setpoint2		
		int i;
		for(i = 0; i< (int)numberOfSetPoints; i++) {
			setPoints2[i][0] = setPoints1[i][0];	setPoints2[i][1] = setPoints1[i][1];	setPoints2[i][2] = setPoints1[i][2];
		}
		
		//Passando o valor do primeiro setpoint, que tambem eh setpoint usado para calcular os angulos de junta para o movimento horizontal do corpo.
		setPoints2[0][0] = -stepSize;	setPoints2[0][1] = -0.04432383389743616;	setPoints2[0][2] = -0.23847004473511105;
		
		//Calculo dos parametros da Interpolacao linear.
		dx = setPoints1[6][0] - setPoints2[0][0];
		dz = setPoints1[6][2] - setPoints2[0][2];
		a = dz/dx;
		b = setPoints1[6][2] - setPoints1[6][0]*a;
		
		//Calculo dos setpoints, posicoes 1 ate 5 na matriz de setpoints2 (ou seja 5 setpoints pois a posicao 0 ja foi definida)
		//dx/6 eh o intervalo entre cada setpoint.
		
		
		for(i = 1;i<6;i++) {
			setPoints2[i][0] = setPoints2[i-1][0] + dx/6;
			setPoints2[i][2] = a*setPoints2[i][0] + b;
		}
		
	}
	
	private void defineSetPoints3(double[][] setPoints, double[][] setPoints1) {
		// Vamos partir do setPoint 7 dos setPoints 1, e a partir dele calcular todos os setPoints anteriores.
		//Sabemos que o setPoint final da perna 3 eh [-0.05, -0.04432383389743616, -0.23847004473511105]
		//Variaveis para interpolacao linar: z = a*x + b
		double b, a, dx, dz;
		//Passando valores do setpoint1 para o setpoint2		
		int i;
		for(i = 0; i< (int)numberOfSetPoints; i++) {
			setPoints[i][0] = setPoints1[i][0];	setPoints[i][1] = setPoints1[i][1];	setPoints[i][2] = setPoints1[i][2];
		}
		
		//Passando o valor do primeiro setpoint, que tambem eh setpoint usado para calcular os angulos de junta para o movimento horizontal do corpo.
		setPoints[0][0] = -2*stepSize;	setPoints[0][1] = -0.04432383389743616;	setPoints[0][2] = -0.23847004473511105;
		
		
		//Calculo dos parametros da Interpolacao linear.
		dx = setPoints1[6][0] - setPoints[0][0];
		dz = setPoints1[6][2] - setPoints[0][2];
		a = dz/dx;
		b = setPoints1[6][2] - setPoints1[6][0]*a;
		
		//Calculo dos setpoints, posicoes 1 ate 5 na matriz de setpoints2 (ou seja 5 setpoints pois a posicao 0 ja foi definida)
		//dx/6 eh o intervalo entre cada setpoint.
		
		
		for(i = 1;i<6;i++) {
			setPoints[i][0] = setPoints[i-1][0] + dx/6;
			setPoints[i][2] = a*setPoints[i][0] + b;
		}
		
		//Muda o sinal da componente y pois agora o corpo esta inclinado para o outro lado
		for(i=0; i<(int) numberOfSetPoints; i++) {
			setPoints[i][1] = - setPoints[i][1];
		}
		
	}
	
	private void stand() {
		//ESSA FUNCAO ATUA COMO UM CONTROLADOR PD QUE APENAS MANTEM O ROBO PARADO EM PE

		
		tau_abdHip0.set(300*(0 - q_abdHip0.getValueAsDouble()) + 3*(0 - qd_abdHip0.getValueAsDouble()));
		tau_abdHip1.set(300*(0 - q_abdHip1.getValueAsDouble()) + 3*(0 - qd_abdHip1.getValueAsDouble()));
		tau_abdHip2.set(300*(0 - q_abdHip2.getValueAsDouble()) + 3*(0 - qd_abdHip2.getValueAsDouble()));
		tau_abdHip3.set(300*(0 - q_abdHip3.getValueAsDouble()) + 3*(0 - qd_abdHip3.getValueAsDouble()));
		
		tau_flexHip0.set(250*(rob.phiY - q_flexHip0.getValueAsDouble()) + 3*(0 - qd_flexHip0.getValueAsDouble()));
		tau_flexHip1.set(250*(rob.phiY - q_flexHip1.getValueAsDouble()) + 3*(0 - qd_flexHip1.getValueAsDouble()));
		tau_flexHip2.set(250*(rob.phiY - q_flexHip2.getValueAsDouble()) + 3*(0 - qd_flexHip2.getValueAsDouble()));
		tau_flexHip3.set(250*(rob.phiY - q_flexHip3.getValueAsDouble()) + 3*(0 - qd_flexHip3.getValueAsDouble()));
		
		tau_flexKnee0.set(300*(-rob.theta - q_flexKnee0.getValueAsDouble()) + 5*(0 - qd_flexKnee0.getValueAsDouble()));
		tau_flexKnee1.set(300*(-rob.theta - q_flexKnee1.getValueAsDouble()) + 5*(0 - qd_flexKnee1.getValueAsDouble()));
		tau_flexKnee2.set(300*(-rob.theta - q_flexKnee2.getValueAsDouble()) + 5*(0 - qd_flexKnee2.getValueAsDouble()));
		tau_flexKnee3.set(300*(-rob.theta - q_flexKnee3.getValueAsDouble()) + 5*(0 - qd_flexKnee3.getValueAsDouble()));
		
		tau_flexAnkle0.set(150*(-rob.psi - q_flexAnkle0.getValueAsDouble()) + 5*(0 - qd_flexAnkle0.getValueAsDouble()));
		tau_flexAnkle1.set(150*(-rob.psi - q_flexAnkle1.getValueAsDouble()) + 5*(0 - qd_flexAnkle1.getValueAsDouble()));
		tau_flexAnkle2.set(150*(-rob.psi - q_flexAnkle2.getValueAsDouble()) + 5*(0 - qd_flexAnkle2.getValueAsDouble()));
		tau_flexAnkle3.set(150*(-rob.psi - q_flexAnkle3.getValueAsDouble()) + 5*(0 - qd_flexAnkle3.getValueAsDouble()));
	

	}
	
	private void leanHip(double angulo) {
		//Se inclinar para a esquerda theta < 0
		//Se inclinar para a direita theta > 0
		
		double k_1 = 250;
		double k_2 = 300;
		double k_3 = 150;
		double k_4 = 300;
		double kd_1 = 3;
		double kd_2 = 5;
		double kd_3 = 5;
		double kd_4 = 3;
		staticLeg0Angles[0] = angulo;
		staticLeg1Angles[0] = angulo;
		staticLeg2Angles[0] = angulo;
		staticLeg3Angles[0] = angulo;
		
		tau_abdHip0.set(k_4*(staticLeg0Angles[0] - q_abdHip0.getValueAsDouble()) + kd_4*(0 - qd_abdHip0.getValueAsDouble()));
		tau_abdHip1.set(k_4*(staticLeg1Angles[0] - q_abdHip1.getValueAsDouble()) + kd_4*(0 - qd_abdHip1.getValueAsDouble()));
		tau_abdHip2.set(k_4*(staticLeg2Angles[0] - q_abdHip2.getValueAsDouble()) + kd_4*(0 - qd_abdHip2.getValueAsDouble()));
		tau_abdHip3.set(k_4*(staticLeg3Angles[0] - q_abdHip3.getValueAsDouble()) + kd_4*(0 - qd_abdHip3.getValueAsDouble()));
		
		tau_flexHip0.set(k_1*(staticLeg0Angles[1] - q_flexHip0.getValueAsDouble()) + kd_1*(0 - qd_flexHip0.getValueAsDouble()));
		tau_flexHip1.set(k_1*(staticLeg1Angles[1] - q_flexHip1.getValueAsDouble()) + kd_1*(0 - qd_flexHip1.getValueAsDouble()));
		tau_flexHip2.set(k_1*(staticLeg2Angles[1] - q_flexHip2.getValueAsDouble()) + kd_1*(0 - qd_flexHip2.getValueAsDouble()));
		tau_flexHip3.set(k_1*(staticLeg3Angles[1] - q_flexHip3.getValueAsDouble()) + kd_1*(0 - qd_flexHip3.getValueAsDouble()));
		
		tau_flexKnee0.set(k_2*(staticLeg0Angles[2] - q_flexKnee0.getValueAsDouble()) + kd_2*(0 - qd_flexKnee0.getValueAsDouble()));
		tau_flexKnee1.set(k_2*(staticLeg3Angles[2] - q_flexKnee1.getValueAsDouble()) + kd_2*(0 - qd_flexKnee1.getValueAsDouble()));
		tau_flexKnee2.set(k_2*(staticLeg3Angles[2] - q_flexKnee2.getValueAsDouble()) + kd_2*(0 - qd_flexKnee2.getValueAsDouble()));
		tau_flexKnee3.set(k_2*(staticLeg3Angles[2] - q_flexKnee3.getValueAsDouble()) + kd_2*(0 - qd_flexKnee3.getValueAsDouble()));
		
		tau_flexAnkle0.set(k_3*(-rob.psi - q_flexAnkle0.getValueAsDouble()) + kd_3*(0 - qd_flexAnkle0.getValueAsDouble()));
		tau_flexAnkle1.set(k_3*(-rob.psi - q_flexAnkle1.getValueAsDouble()) + kd_3*(0 - qd_flexAnkle1.getValueAsDouble()));
		tau_flexAnkle2.set(k_3*(-rob.psi - q_flexAnkle2.getValueAsDouble()) + kd_3*(0 - qd_flexAnkle2.getValueAsDouble()));
		tau_flexAnkle3.set(k_3*(-rob.psi - q_flexAnkle3.getValueAsDouble()) + kd_3*(0 - qd_flexAnkle3.getValueAsDouble()));
		
		
		
		
	}
	
	private void trajParabol(double [][]sp, double []hipToAnkle, double step, double nSPs) {
		//[][]sp --> vetor que guarda as coordenadas dos set points
		//step --> comprimento do passo
		//nSPs --> numero de SetPoints
		//sabemos q a eq sera:f(x) = z = x^2 - Sx
		double S = -step;
		double cont;
		
		sp[0][0] = hipToAnkle[0];
		sp[0][1] = hipToAnkle[1];
		sp[0][2] = hipToAnkle[2];
		
		for(cont=1;cont<nSPs;cont++) {
			sp[(int)cont][0] = hipToAnkle[0] + cont*(step/(nSPs-1));
			sp[(int)cont][1] = hipToAnkle[1];
			sp[(int)cont][2] = hipToAnkle[2] - (Math.pow(cont*step/(nSPs-1),2) - step*cont*step/(nSPs-1));
		}
		
		System.out.println("----- SETPOINTS PARABOLA ------");
		for(cont=0;cont<nSPs;cont++) {
			System.out.println("[" + sp[(int)cont][0] + ", " + sp[(int)cont][1] + ", " + sp[(int)cont][2] + "]");
		}
		System.out.println("----- SETPOINTS PARABOLA ------");
		
		double []angles = new double[3];
		System.out.println("----- CINEMATICA INVERSA ------");
		for(cont=0;cont<nSPs;cont++) {
			inverseKinematics(angles, setPoints1[(int)cont]);
			System.out.println("[" + angles[0] + ", " + angles[1] + ", " + angles[2] + "]");
		}
		System.out.println("----- CINEMATICA INVERSA ------");
	}
	
	private double myAtan(double x, double z) {
		double arcTan = 0 ;
		
		if(x<0 && z<0) {
			arcTan = Math.atan(Math.abs(x)/Math.abs(z));
		}
		if(x<0 && z>0) {
			arcTan = -Math.atan(Math.abs(x)/Math.abs(z));
		}
		
		return arcTan;
	}
	
	public void COMandCOMVelocity() {
		
		Point3D instCOMPosition = new Point3D();
		Vector3D tempLinearMomentum = new Vector3D();
	    Vector3D tempAngularMomentum = new Vector3D();
	    /*
	       * CoM and CoM velocity in WorldFrame
	    */
	    
	    COM.set(tempCOMPosition);
	    linearMomentum.set(tempLinearMomentum);
	    angularMomentum.set(tempAngularMomentum);
	    comVelocity.set(tempLinearMomentum);
	    
	}
	
	
   public void getCOMdata(){

	      Point3D tempCOMPosition = new Point3D();
	      Vector3D tempLinearMomentum = new Vector3D();
	      Vector3D tempAngularMomentum = new Vector3D();
	      /*
	       * CoM and CoM velocity in WorldFrame
	       */
	      
	      double totalMass = rob.computeCenterOfMass(tempCOMPosition);/*rob.computeCOMMomentum(tempCOMPosition, tempLinearMomentum, tempAngularMomentum);*/
	      COM.set(tempCOMPosition);
	      centroDM.translate(tempCOMPosition.getX(), tempCOMPosition.getY(), tempCOMPosition.getZ());
	      rob.computeAngularMomentum(tempAngularMomentum);
	      
	      linearMomentum.set(tempLinearMomentum);
	      angularMomentum.set(tempAngularMomentum);
	      tempLinearMomentum.scale(1.0 / totalMass);
	      comVelocity.set(tempLinearMomentum);
	      /*
	       * CoM to Foot error
	       */
	      //comToFootError.sub(COM.getFrameTuple(), footLocation.getFrameTuple());
	      //comToFootError.setZ(0.0);
   }

   private void getAnklePositions() {
	   	FrameVector3D ankle0 = new FrameVector3D();
		FrameVector3D ankle1 = new FrameVector3D();
		FrameVector3D ankle2 = new FrameVector3D();
		FrameVector3D ankle3 = new FrameVector3D();
		
		flexAnkle0.getTranslationToWorld(ankle0);
		flexAnkle1.getTranslationToWorld(ankle1);
		flexAnkle2.getTranslationToWorld(ankle2);
		flexAnkle3.getTranslationToWorld(ankle3);
		
		x_ankle0.set(ankle0.getX());	y_ankle0.set(ankle0.getY());	z_ankle0.set(ankle0.getZ());
		x_ankle1.set(ankle1.getX());	y_ankle1.set(ankle1.getY());	z_ankle1.set(ankle1.getZ());
		x_ankle2.set(ankle2.getX());	y_ankle2.set(ankle2.getY());	z_ankle2.set(ankle2.getZ());
		x_ankle3.set(ankle3.getX());	y_ankle3.set(ankle3.getY());	z_ankle3.set(ankle3.getZ());
		
		
   }
   
   
}
