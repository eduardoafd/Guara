package guara;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;


public class GuaraYoVariablesDefinition {

	protected GuaraRobot rob;
	// floating joint - corpo

	public YoDouble q_bodyJoint_x, q_bodyJoint_y, q_bodyJoint_z, qd_bodyJoint_x, qd_bodyJoint_y, qd_bodyJoint_z,
			qdd_bodyJoint_x, qdd_bodyJoint_y, qdd_bodyJoint_z, q_bodyJoint_qs, q_bodyJoint_qx, q_bodyJoint_qy,
			q_bodyJoint_qz, qd_bodyJoint_wx, qd_bodyJoint_wy, qd_bodyJoint_wz, qdd_bodyJoint_wx, qdd_bodyJoint_wy,
			qdd_bodyJoint_wz;

	// universal joints- quadris

	// quadril perna 0

	public YoDouble q_abdHip0X, qd_abdHip0X, qdd_abdHip0X, tau_abdHip0X, q_abdHip0Y, qd_abdHip0Y, qdd_abdHip0Y,
			tau_abdHip0Y;

	// quadril perna 1

	public YoDouble q_abdHip1X, qd_abdHip1X, qdd_abdHip1X, tau_abdHip1X, q_abdHip1Y, qd_abdHip1Y, qdd_abdHip1Y,
			tau_abdHip1Y;

	// quadril perna 2

	public YoDouble q_abdHip2X, qd_abdHip2X, qdd_abdHip2X, tau_abdHip2X, q_abdHip2Y, qd_abdHip2Y, qdd_abdHip2Y,
			tau_abdHip2Y;

	// quadril perna 3

	public YoDouble q_abdHip3X, qd_abdHip3X, qdd_abdHip3X, tau_abdHip3X, q_abdHip3Y, qd_abdHip3Y, qdd_abdHip3Y,
			tau_abdHip3Y;

	// pin joints - joelhos

	// joelho perna 0

	public YoDouble q_flexKnee0, qd_flexKnee0, qdd_flexKnee0, tau_flexKnee0;

	// joelho perna 1

	public YoDouble q_flexKnee1, qd_flexKnee1, qdd_flexKnee1, tau_flexKnee1;

	// joelho perna 2

	public YoDouble q_flexKnee2, qd_flexKnee2, qdd_flexKnee2, tau_kneeFlex2;

	// joelho perna 3

	public YoDouble q_flexKnee3, qd_flexKnee3, qdd_flexKnee3, tau_flexKnee3;

	// pin joints - tornozelos

	// tornozelo perna 0

	public YoDouble q_flexAnkle0, qd_flexAnkle0, qdd_flexAnkle0, tau_flexAnkle0;

	// tornozelo perna 1

	public YoDouble q_flexAnkle1, qd_flexAnkle1, qdd_flexAnkle1, tau_flexAnkle1;

	// tornozelo perna 2

	public YoDouble q_flexAnkle2, qd_flexAnkle2, qdd_flexAnkle2, tau_flexAnkle2;

	// tornozelo perna 3

	public YoDouble q_flexAnkle3, qd_flexAnkle3, qdd_flexAnkle3, tau_flexAnkle3;

	private final YoVariableRegistry registry = new YoVariableRegistry("guaraYoVariables");

	// construtor
	public GuaraYoVariablesDefinition() {
	}

	public GuaraYoVariablesDefinition(GuaraRobot rob) {

		// this.rob = rob;

		// variáveis da junta flutuante do quadril

		q_bodyJoint_x = (YoDouble) rob.getVariable("q_bodyJoint_x");

		q_bodyJoint_x = (YoDouble) rob.getVariable("q_bodyJoint_x");
		q_bodyJoint_y = (YoDouble) rob.getVariable("q_bodyJoint_y");
		q_bodyJoint_z = (YoDouble) rob.getVariable("q_bodyJoint_z");
		qd_bodyJoint_x = (YoDouble) rob.getVariable("qd_bodyJoint_x");
		qd_bodyJoint_y = (YoDouble) rob.getVariable("qd_bodyJoint_y");
		qd_bodyJoint_z = (YoDouble) rob.getVariable("qd_bodyJoint_z");
		qdd_bodyJoint_x = (YoDouble) rob.getVariable("qdd_bodyJoint_x");
		qdd_bodyJoint_y = (YoDouble) rob.getVariable("qdd_bodyJoint_y");
		qdd_bodyJoint_z = (YoDouble) rob.getVariable("qdd_bodyJoint_z");
		q_bodyJoint_qs = (YoDouble) rob.getVariable("q_bodyJoint_qs");
		q_bodyJoint_qx = (YoDouble) rob.getVariable("q_bodyJoint_qx");
		q_bodyJoint_qy = (YoDouble) rob.getVariable("q_bodyJoint_qy");
		q_bodyJoint_qz = (YoDouble) rob.getVariable("q_bodyJoint_qz");
		qd_bodyJoint_wx = (YoDouble) rob.getVariable("qd_bodyJoint_wx");
		qd_bodyJoint_wy = (YoDouble) rob.getVariable("qd_bodyJoint_wy");
		qd_bodyJoint_wz = (YoDouble) rob.getVariable("qd_bodyJoint_wz");
		qdd_bodyJoint_wx = (YoDouble) rob.getVariable("qdd_bodyJoint_wx");
		qdd_bodyJoint_wy = (YoDouble) rob.getVariable("qdd_bodyJoint_wy");
		qdd_bodyJoint_wz = (YoDouble) rob.getVariable("qdd_bodyJoint_wz");

		// variáveis da junta universal do quadril - perna 0

		q_abdHip0X = (YoDouble) rob.getVariable("q_abdHip0X");
		qd_abdHip0X = (YoDouble) rob.getVariable("qd_abdHip0X");
		qdd_abdHip0X = (YoDouble) rob.getVariable("qdd_abdHip0X");
		tau_abdHip0X = (YoDouble) rob.getVariable("tau_abdHip0X");
		q_abdHip0Y = (YoDouble) rob.getVariable("q_abdHip0Y");
		qd_abdHip0Y = (YoDouble) rob.getVariable("qd_abdHip0Y");
		qdd_abdHip0Y = (YoDouble) rob.getVariable("qdd_abdHip0Y");
		tau_abdHip0Y = (YoDouble) rob.getVariable("tau_abdHip0Y");

		// variáveis da junta universal do quadril - perna 1

		q_abdHip1X = (YoDouble) rob.getVariable("q_abdHip1X");
		qd_abdHip1X = (YoDouble) rob.getVariable("qd_abdHip1X");
		qdd_abdHip1X = (YoDouble) rob.getVariable("qdd_abdHip1X");
		tau_abdHip1X = (YoDouble) rob.getVariable("tau_abdHip1X");
		q_abdHip1Y = (YoDouble) rob.getVariable("q_abdHip1Y");
		qd_abdHip1Y = (YoDouble) rob.getVariable("qd_abdHip1Y");
		qdd_abdHip1Y = (YoDouble) rob.getVariable("qdd_abdHip1Y");
		tau_abdHip1Y = (YoDouble) rob.getVariable("tau_abdHip1Y");

		// variáveis da junta universal do quadril - perna 2

		q_abdHip2X = (YoDouble) rob.getVariable("q_abdHip2X");
		qd_abdHip2X = (YoDouble) rob.getVariable("qd_abdHip2X");
		qdd_abdHip2X = (YoDouble) rob.getVariable("qdd_abdHip2X");
		tau_abdHip2X = (YoDouble) rob.getVariable("tau_abdHip2X");
		q_abdHip2Y = (YoDouble) rob.getVariable("q_abdHip2Y");
		qd_abdHip2Y = (YoDouble) rob.getVariable("qd_abdHip2Y");
		qdd_abdHip2Y = (YoDouble) rob.getVariable("qdd_abdHip2Y");
		tau_abdHip2Y = (YoDouble) rob.getVariable("tau_abdHip2Y");

		// variáveis da junta universal do quadril - perna 3

		q_abdHip3X = (YoDouble) rob.getVariable("q_abdHip3X");
		qd_abdHip3X = (YoDouble) rob.getVariable("qd_abdHip3X");
		qdd_abdHip3X = (YoDouble) rob.getVariable("qdd_abdHip3X");
		tau_abdHip3X = (YoDouble) rob.getVariable("tau_abdHip3X");
		q_abdHip3Y = (YoDouble) rob.getVariable("q_abdHip3Y");
		qd_abdHip3Y = (YoDouble) rob.getVariable("qd_abdHip3Y");
		qdd_abdHip3Y = (YoDouble) rob.getVariable("qdd_abdHip3Y");
		tau_abdHip3Y = (YoDouble) rob.getVariable("tau_abdHip3Y");

		// variáveis da pin joint do joelho - perna 0

		q_flexKnee0 = (YoDouble) rob.getVariable("q_flexKnee0");
		qd_flexKnee0 = (YoDouble) rob.getVariable("qd_flexKnee0");
		qdd_flexKnee0 = (YoDouble) rob.getVariable("qdd_flexKnee0");
		tau_flexKnee0 = (YoDouble) rob.getVariable("tau_flexKnee0");

		// variáveis da pin joint do joelho - perna 1

		q_flexKnee1 = (YoDouble) rob.getVariable("q_flexKnee1");
		qd_flexKnee1 = (YoDouble) rob.getVariable("qd_flexKnee1");
		qdd_flexKnee1 = (YoDouble) rob.getVariable("qdd_flexKnee1");
		tau_flexKnee1 = (YoDouble) rob.getVariable("tau_flexKnee1");
		//
		// //variáveis da pin joint do joelho - perna 2
		//
		q_flexKnee2 = (YoDouble) rob.getVariable("q_flexKnee2");
		qd_flexKnee2 = (YoDouble) rob.getVariable("qd_flexKnee2");
		qdd_flexKnee2 = (YoDouble) rob.getVariable("qdd_flexKnee2");
		tau_kneeFlex2 = (YoDouble) rob.getVariable("tau_flexKnee2");
		//
		// //variáveis da pin joint do joelho - perna 3
		//
		q_flexKnee3 = (YoDouble) rob.getVariable("q_flexKnee3");
		qd_flexKnee3 = (YoDouble) rob.getVariable("qd_flexKnee3");
		qdd_flexKnee3 = (YoDouble) rob.getVariable("qdd_flexKnee3");
		tau_flexKnee3 = (YoDouble) rob.getVariable("tau_flexKnee3");
		//
		// //variáveis da pin joint do tornozelo - perna 0
		//
		q_flexAnkle0 = (YoDouble) rob.getVariable("q_flexAnkle0");
		qd_flexAnkle0 = (YoDouble) rob.getVariable("qd_flexKnee0");
		qdd_flexAnkle0 = (YoDouble) rob.getVariable("qdd_flexKnee0");
		tau_flexAnkle0 = (YoDouble) rob.getVariable("tau_flexKnee0");
		//
		// //variáveis da pin joint do tornozelo - perna 1
		//
		q_flexAnkle1 = (YoDouble) rob.getVariable("q_flexAnkle1");
		qd_flexKnee1 = (YoDouble) rob.getVariable("qd_flexAnkle1");
		qdd_flexAnkle1 = (YoDouble) rob.getVariable("qdd_flexAnkle1");
		tau_flexAnkle1 = (YoDouble) rob.getVariable("tau_flexAnkle1");
		//
		// //variáveis da pin joint do tornozelo - perna 2
		//
		q_flexAnkle2 = (YoDouble) rob.getVariable("q_flexAnkle2");
		qd_flexAnkle2 = (YoDouble) rob.getVariable("qd_flexAnkle2");
		qdd_flexAnkle2 = (YoDouble) rob.getVariable("qdd_flexAnkle2");
		tau_flexAnkle2 = (YoDouble) rob.getVariable("tau_flexAnkle2");
		//
		// //variáveis da pin joint do tornozelo - perna 3
		//
		q_flexAnkle3 = (YoDouble) rob.getVariable("q_flexAnkle3");
		qd_flexAnkle3 = (YoDouble) rob.getVariable("qd_flexAnkle3");
		qdd_flexAnkle3 = (YoDouble) rob.getVariable("qdd_flexAnkle3");
		tau_flexAnkle3 = (YoDouble) rob.getVariable("tau_flexAnkle3");
	}

}
