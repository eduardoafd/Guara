package guara;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class GuaraController /* extends GuaraYoVariablesDefinition */ implements RobotController
{
   private GuaraRobot robot;

   //	public GuaraYoVariablesDefinition yo;
   public GuaraWaveGait a3;

   private final YoVariableRegistry registry = new YoVariableRegistry("guaraController");

   // constantes do controlador da perna

   double Kp0, Kd0, Ki0, // junta 0
         Kp1, Kd1, Ki1, // junta 1
         Kp2, Kd2, Ki2, // junta 2
         Kp3, Kd3, Ki3; // junta 3

   private YoDouble abdFlexHip0, abdFlexHip1, abdFlexHip2, abdFlexHip3;
   private YoDouble tau_abdHip0, tau_abdHip1, tau_abdHip2, tau_abdHip3, q_abdHip0, q_abdHip1, q_abdHip2, q_abdHip3, qd_abdHip0, qd_abdHip1, qd_abdHip2,
         qd_abdHip3;
   private YoDouble tau_flexHip0, tau_flexHip1, tau_flexHip2, tau_flexHip3, q_flexHip0, q_flexHip1, q_flexHip2, q_flexHip3, qd_flexHip0, qd_flexHip1,
         qd_flexHip2, qd_flexHip3;
   //   private YoDouble qd_abdHip0, qd_flexAnkle0;
   private YoDouble tau_flexKnee0, tau_flexKnee1, tau_flexKnee2, tau_flexKnee3, q_flexKnee0, q_flexKnee1, q_flexKnee2, q_flexKnee3, qd_flexKnee0, qd_flexKnee1,
         qd_flexKnee2, qd_flexKnee3;
   private YoDouble tau_flexAnkle0, tau_flexAnkle1, tau_flexAnkle2, tau_flexAnkle3, q_flexAnkle0, q_flexAnkle1, q_flexAnkle2, q_flexAnkle3, qd_flexAnkle0,
         qd_flexAnkle1, qd_flexAnkle2, qd_flexAnkle3;

   // vari�veis de set point = posi��o

   double[][] spTeta = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}; // declara e inicia
   double[][] tetaAt = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
   double[][] inAn = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}; // integral anterior uma perna
   double[][] inAt = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}; // integral atual uma perna
   double[][] xyz = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};// coordenadas

   // set points counter

   int i = 0;

   public double thetad;

   public GuaraController(GuaraRobot robot)
   {

      abdFlexHip0 = (YoDouble) robot.getVariable("abdFlexHip0");
      abdFlexHip1 = (YoDouble) robot.getVariable("abdFlexHip1");
      abdFlexHip2 = (YoDouble) robot.getVariable("abdFlexHip2");
      abdFlexHip3 = (YoDouble) robot.getVariable("abdFlexHip3");
      tau_abdHip0 = (YoDouble) robot.getVariable("tau_abdHip0");
      tau_abdHip1 = (YoDouble) robot.getVariable("tau_abdHip1");
      tau_abdHip2 = (YoDouble) robot.getVariable("tau_abdHip2");
      tau_abdHip3 = (YoDouble) robot.getVariable("tau_abdHip3");
      q_abdHip0 = (YoDouble) robot.getVariable("q_abdHip0");
      q_abdHip1 = (YoDouble) robot.getVariable("q_abdHip1");
      q_abdHip2 = (YoDouble) robot.getVariable("q_abdHip2");
      q_abdHip3 = (YoDouble) robot.getVariable("q_abdHip3");
      qd_abdHip0 = (YoDouble) robot.getVariable("qd_abdHip0");
      qd_abdHip1 = (YoDouble) robot.getVariable("qd_abdHip1");
      qd_abdHip2 = (YoDouble) robot.getVariable("qd_abdHip2");
      qd_abdHip3 = (YoDouble) robot.getVariable("qd_abdHip3");

      tau_flexHip0 = (YoDouble) robot.getVariable("tau_flexHip0");
      tau_flexHip1 = (YoDouble) robot.getVariable("tau_flexHip1");
      tau_flexHip2 = (YoDouble) robot.getVariable("tau_flexHip2");
      tau_flexHip3 = (YoDouble) robot.getVariable("tau_flexHip3");
      q_flexHip0 = (YoDouble) robot.getVariable("q_flexHip0");
      q_flexHip1 = (YoDouble) robot.getVariable("q_flexHip1");
      q_flexHip2 = (YoDouble) robot.getVariable("q_flexHip2");
      q_flexHip3 = (YoDouble) robot.getVariable("q_flexHip3");
      qd_flexHip0 = (YoDouble) robot.getVariable("qd_flexHip0");
      qd_flexHip1 = (YoDouble) robot.getVariable("qd_flexHip1");
      qd_flexHip2 = (YoDouble) robot.getVariable("qd_flexHip2");
      qd_flexHip3 = (YoDouble) robot.getVariable("qd_flexHip3");

      tau_flexKnee0 = (YoDouble) robot.getVariable("tau_flexKnee0");
      tau_flexKnee1 = (YoDouble) robot.getVariable("tau_flexKnee1");
      tau_flexKnee2 = (YoDouble) robot.getVariable("tau_flexKnee2");
      tau_flexKnee3 = (YoDouble) robot.getVariable("tau_flexKnee3");
      q_flexKnee0 = (YoDouble) robot.getVariable("q_flexKnee0");
      q_flexKnee1 = (YoDouble) robot.getVariable("q_flexKnee1");
      q_flexKnee2 = (YoDouble) robot.getVariable("q_flexKnee2");
      q_flexKnee3 = (YoDouble) robot.getVariable("q_flexKnee3");
      qd_flexKnee0 = (YoDouble) robot.getVariable("qd_flexKnee0");
      qd_flexKnee1 = (YoDouble) robot.getVariable("qd_flexKnee1");
      qd_flexKnee2 = (YoDouble) robot.getVariable("qd_flexKnee2");
      qd_flexKnee3 = (YoDouble) robot.getVariable("qd_flexKnee3");

      tau_flexAnkle0 = (YoDouble) robot.getVariable("tau_flexAnkle0");
      tau_flexAnkle1 = (YoDouble) robot.getVariable("tau_flexAnkle1");
      tau_flexAnkle2 = (YoDouble) robot.getVariable("tau_flexAnkle2");
      tau_flexAnkle3 = (YoDouble) robot.getVariable("tau_flexAnkle3");
      q_flexAnkle0 = (YoDouble) robot.getVariable("q_flexAnkle0");
      q_flexAnkle1 = (YoDouble) robot.getVariable("q_flexAnkle1");
      q_flexAnkle2 = (YoDouble) robot.getVariable("q_flexAnkle2");
      q_flexAnkle3 = (YoDouble) robot.getVariable("q_flexAnkle3");
      qd_flexAnkle0 = (YoDouble) robot.getVariable("qd_flexAnkle0");
      qd_flexAnkle1 = (YoDouble) robot.getVariable("qd_flexAnkle1");
      qd_flexAnkle2 = (YoDouble) robot.getVariable("qd_flexAnkle2");
      qd_flexAnkle3 = (YoDouble) robot.getVariable("qd_flexAnkle3");

      //		System.out.println("guaraController");
      this.robot = robot;

      a3 = new GuaraWaveGait(128);
      assert a3 != null;
      // System.out.println("a3==null");
      // System.out.println(a3 == null);

      initControl();
      System.out.println("saiu initcontrol");

   }

   public void initControl()
   {

      // inicia com pernas na vertical

      System.out.println("initControl");

      //legs' ground contact coordinates

      xyz[0][0] = 0.0;
      xyz[0][1] = 0.0;
      xyz[0][2] = -0.3; // robot height with straighten legs

      xyz[1][0] = 0.0;
      xyz[1][1] = 0.0;
      xyz[1][2] = -0.3; // robot height with straighten legs

      xyz[2][0] = 0.0;
      xyz[2][1] = 0.0;
      xyz[2][2] = -0.3; // robot height with straighten legs

      xyz[3][0] = 0.0;
      xyz[3][1] = 0.0;
      xyz[3][2] = -0.3; // robot height with straighten legs

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

      tetaAt[0][0] = qd_abdHip0.getDoubleValue();
      tetaAt[0][1] = -Math.PI / 12;
      tetaAt[0][2] = Math.PI / 12;
      tetaAt[0][3] = qd_flexAnkle0.getDoubleValue();

      tau_flexHip0.set(10.00 * (robot.theta - q_flexHip0.getValueAsDouble()));
      tau_flexKnee0.set(10.00 * (robot.theta - q_flexKnee0.getValueAsDouble()));
      tau_flexAnkle0.set(10.00 * (robot.theta - q_flexAnkle0.getValueAsDouble()));

      tau_abdHip1.set(10.00 * (robot.theta - q_abdHip1.getValueAsDouble()));
      tau_flexHip1.set(10.00 * (robot.theta - q_flexHip1.getValueAsDouble()));
      tau_flexKnee1.set(10.00 * (robot.theta - q_flexKnee1.getValueAsDouble()));
      tau_flexAnkle1.set(10.00 * (robot.theta - q_flexAnkle1.getValueAsDouble()));

      tau_abdHip2.set(10.00 * (robot.theta - q_abdHip2.getValueAsDouble()));
      tau_flexHip2.set(10.00 * (robot.theta - q_flexHip2.getValueAsDouble()));
      tau_flexKnee2.set(10.00 * (robot.theta - q_flexKnee2.getValueAsDouble()));
      tau_flexAnkle2.set(10.00 * (robot.theta - q_flexAnkle2.getValueAsDouble()));

      tau_abdHip3.set(10.00 * (robot.theta - q_abdHip3.getValueAsDouble()));
      tau_flexHip3.set(10.00 * (robot.theta - q_flexHip3.getValueAsDouble()));
      tau_flexKnee3.set(10.00 * (robot.theta - q_flexKnee3.getValueAsDouble()));
      tau_flexAnkle3.set(10.00 * (robot.theta - q_flexAnkle3.getValueAsDouble()));

//      robot.robotConfig1(abdFlexHip0, q_flexKnee0, q_flexAnkle0, abdFlexHip1, q_flexKnee1, q_flexAnkle1, abdFlexHip2, q_flexKnee2, q_flexAnkle2, abdFlexHip3, q_flexKnee3,
//                   flexAnkle3);

   }

   public void doControl()
   {

      System.out.println("entrou docontrol");

      //double valortau = (100.00*(rob.theta - yo.qd_flexKnee0.getValueAsDouble()));

      tau_abdHip0.set(10.00 * (robot.theta - q_abdHip0.getValueAsDouble()));
      tau_flexHip0.set(10.00 * (robot.theta - q_flexHip0.getValueAsDouble()));
      tau_flexKnee0.set(10.00 * (robot.theta - q_flexKnee0.getValueAsDouble()));
      tau_flexAnkle0.set(10.00 * (robot.theta - q_flexAnkle0.getValueAsDouble()));

      tau_abdHip1.set(10.00 * (robot.theta - q_abdHip1.getValueAsDouble()));
      tau_flexHip1.set(10.00 * (robot.theta - q_flexHip1.getValueAsDouble()));
      tau_flexKnee1.set(10.00 * (robot.theta - q_flexKnee1.getValueAsDouble()));
      tau_flexAnkle1.set(10.00 * (robot.theta - q_flexAnkle1.getValueAsDouble()));

      tau_abdHip2.set(10.00 * (robot.theta - q_abdHip2.getValueAsDouble()));
      tau_flexHip2.set(10.00 * (robot.theta - q_flexHip2.getValueAsDouble()));
      tau_flexKnee2.set(10.00 * (robot.theta - q_flexKnee2.getValueAsDouble()));
      tau_flexAnkle2.set(10.00 * (robot.theta - q_flexAnkle2.getValueAsDouble()));

      tau_abdHip3.set(10.00 * (robot.theta - q_abdHip3.getValueAsDouble()));
      tau_flexHip3.set(10.00 * (robot.theta - q_flexHip3.getValueAsDouble()));
      tau_flexKnee3.set(10.00 * (robot.theta - q_flexKnee3.getValueAsDouble()));
      tau_flexAnkle3.set(10.00 * (robot.theta - q_flexAnkle3.getValueAsDouble()));

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
