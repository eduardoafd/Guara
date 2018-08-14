package guara;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class GuaraController /* extends GuaraYoVariablesDefinition */ implements RobotController
{
   private GuaraRobot rob;

   //	public GuaraYoVariablesDefinition yo;
   public GuaraWaveGait a3;

   private final YoVariableRegistry registry = new YoVariableRegistry("guaraController");

   // constantes do controlador da perna

   double Kp0, Kd0, Ki0, // junta 0
         Kp1, Kd1, Ki1, // junta 1
         Kp2, Kd2, Ki2, // junta 2
         Kp3, Kd3, Ki3; // junta 3

   private YoDouble tau_flexKnee0, tau_flexKnee1, tau_flexKnee2, tau_flexKnee3, q_flexKnee0, q_flexKnee1, q_flexKnee2, q_flexKnee3, q_abdHip0X, q_flexAnkle0;

   // vari�veis de set point = posi��o

   double[][] spTeta = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}; // declara e inicia
   double[][] tetaAt = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
   double[][] inAn = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}; // integral anterior uma perna
   double[][] inAt = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}; // integral atual uma perna
   double[][] xyz = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};// coordenadas
   // 4
   // set points counter

   int i = 0;

   public double thetad;

   public GuaraController(GuaraRobot robot)
   { //, String name) {

      tau_flexKnee0 = (YoDouble) robot.getVariable("tau_flexKnee0");
      tau_flexKnee1 = (YoDouble) robot.getVariable("tau_flexKnee1");
      tau_flexKnee2 = (YoDouble) robot.getVariable("tau_flexKnee2");
      tau_flexKnee3 = (YoDouble) robot.getVariable("tau_flexKnee3");
      q_flexKnee0 = (YoDouble) robot.getVariable("q_flexKnee0");
      q_flexKnee1 = (YoDouble) robot.getVariable("q_flexKnee1");
      q_flexKnee2 = (YoDouble) robot.getVariable("q_flexKnee2");
      q_flexKnee3 = (YoDouble) robot.getVariable("q_flexKnee3");
      q_abdHip0X = (YoDouble) robot.getVariable("q_abdHip0X");
      q_flexAnkle0 = (YoDouble) robot.getVariable("q_flexAnkle0");

      //	   super(rob);
      //		System.out.println("guaraController");
      //	      this.name = name;
      this.rob = robot;

      a3 = new GuaraWaveGait(128);
      assert a3 != null;
      // System.out.println("a3==null");
      // System.out.println(a3 == null);

      initControl();
      System.out.println("saiu initcontrol");

      //		yo = new GuaraYoVariablesDefinition();
   }

   public void initControl()
   {

      // inicia com pernas na vertical

      System.out.println("initControl");

      //legs' ground contact coordinates

      xyz[0][0] = 0.0;
      xyz[0][1] = 0.0;
      xyz[0][2] = -0.3; // robot height with straighten kegs

      xyz[1][0] = 0.0;
      xyz[1][1] = 0.0;
      xyz[1][2] = -0.3; // robot height with straighten kegs

      xyz[2][0] = 0.0;
      xyz[2][1] = 0.0;
      xyz[2][2] = -0.3; // robot height with straighten kegs

      xyz[3][0] = 0.0;
      xyz[3][1] = 0.0;
      xyz[3][2] = -0.3; // robot height with straighten kegs

      // Constantes de integra��o por perna,
      // inicialmente iguais para todas as juntas

      Kp0 = 1.0;
      Kd0 = 1.0;
      Ki0 = 1.0; // junta 0
      Kp1 = 1.0; // junta 1
      Kd1 = 1.0;
      Ki1 = 1.0;
      Kp2 = 2.0; // junta 2
      Kd2 = 2.0;
      Ki2 = 2.0;
      Kp3 = 1.0; // junta3
      Kd3 = 1.0;
      Ki3 = 1.0;

      // start tilt

      tetaAt[0][0] = q_abdHip0X.getDoubleValue();
      tetaAt[0][1] = -Math.PI / 12;
      tetaAt[0][2] = Math.PI / 12;
      tetaAt[0][3] = q_flexAnkle0.getDoubleValue();

   }

   public void doControl()
   {
      /*
       * // TODO Auto-generated method stub System.out.println("doControl"); //
       * legs' joint variables // tetaAt[0][0] = q_abdHip0X.getDoubleValue(); //
       * tetaAt[0][1] = q_abdHip0Y.getDoubleValue(); // tetaAt[0][2] =
       * q_flexKnee0.getDoubleValue(); // tetaAt[0][3] =
       * q_flexAnkle0.getDoubleValue(); tetaAt[1][0] =
       * q_abdHip1X.getDoubleValue(); tetaAt[1][1] =
       * q_abdHip1Y.getDoubleValue(); tetaAt[1][2] =
       * q_flexKnee1.getDoubleValue(); tetaAt[1][3] =
       * q_flexAnkle1.getDoubleValue(); tetaAt[2][0] =
       * q_abdHip2X.getDoubleValue(); tetaAt[2][1] =
       * q_abdHip2Y.getDoubleValue(); tetaAt[2][2] =
       * q_flexKnee2.getDoubleValue(); tetaAt[2][3] =
       * q_flexAnkle2.getDoubleValue(); tetaAt[3][0] =
       * q_abdHip3X.getDoubleValue(); tetaAt[3][1] =
       * q_abdHip3Y.getDoubleValue(); tetaAt[3][2] =
       * q_flexKnee3.getDoubleValue(); tetaAt[3][3] =
       * q_flexAnkle3.getDoubleValue(); // legs' integral terms inAt[0][0] =
       * (spTeta[0][0] + q_abdHip0X.getDoubleValue()) / 2; inAt[0][1] =
       * (spTeta[0][1] + q_abdHip0Y.getDoubleValue()) / 2; inAt[0][2] =
       * (spTeta[0][2] + q_flexKnee0.getDoubleValue()) / 2; inAt[0][3] =
       * (spTeta[0][3] + q_flexAnkle0.getDoubleValue()) / 2; inAt[1][0] =
       * (spTeta[1][0] + q_abdHip1X.getDoubleValue()) / 2; inAt[1][1] =
       * (spTeta[1][1] + q_abdHip1Y.getDoubleValue()) / 2; inAt[1][2] =
       * (spTeta[1][2] + q_flexKnee1.getDoubleValue()) / 2; inAt[1][3] =
       * (spTeta[1][3] + q_flexAnkle1.getDoubleValue()) / 2; inAt[2][0] =
       * (spTeta[2][0] + q_abdHip2X.getDoubleValue()) / 2; inAt[2][1] =
       * (spTeta[2][1] + q_abdHip2Y.getDoubleValue()) / 2; inAt[2][2] =
       * (spTeta[2][2] + q_flexKnee2.getDoubleValue()) / 2; inAt[2][3] =
       * (spTeta[2][3] + q_flexAnkle2.getDoubleValue()) / 2; inAt[3][0] =
       * (spTeta[3][0] + q_abdHip3X.getDoubleValue()) / 2; inAt[3][1] =
       * (spTeta[3][1] + q_abdHip3Y.getDoubleValue()) / 2; inAt[3][2] =
       * (spTeta[3][2] + q_flexKnee3.getDoubleValue()) / 2; inAt[3][3] =
       * (spTeta[3][3] + q_flexAnkle3.getDoubleValue()) / 2; // leg 0 joints
       * control spTeta[0] = a3.legJoints(0, 1, xyz[0][0], xyz[0][1],
       * xyz[0][2]); spTeta[0] = a3.legJoints(0, 1, xyz[0][0], xyz[0][1],
       * xyz[0][2]); spTeta[0] = a3.legJoints(0, 1, xyz[0][0], xyz[0][1],
       * xyz[0][2]); spTeta[0] = a3.legJoints(0, 1, xyz[0][0], xyz[0][1],
       * xyz[0][2]); yo.tau_abdHip0X.set( +Kp0 * (spTeta[0][0] - tetaAt[0][0]) +
       * Kd0 * yo.qd_abdHip0X.getDoubleValue() + Ki0 * (inAt[0][0] +
       * inAn[0][0])); inAt[0][0] = inAn[0][0]; // last integral
       */
      System.out.println("entrou docontrol");

      //double valortau = (100.00*(rob.theta - yo.q_flexKnee0.getValueAsDouble()));

      tau_flexKnee0.set(10.00 * (rob.theta - q_flexKnee0.getValueAsDouble()));
      tau_flexKnee1.set(10.00 * (rob.theta - q_flexKnee1.getValueAsDouble()));
      tau_flexKnee2.set(10.00 * (rob.theta - q_flexKnee2.getValueAsDouble()));
      tau_flexKnee3.set(10.00 * (rob.theta - q_flexKnee3.getValueAsDouble()));

      System.out.println("saiu docontrol");

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

}
