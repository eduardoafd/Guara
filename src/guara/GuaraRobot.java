package guara;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
//import us.ihmc.robotics.Axis;
import us.ihmc.euclid.Axis;
//SimulationConstructionSet;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UniversalJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class GuaraRobot extends Robot
{
   private final ArrayList<GroundContactPoint> groundContactPoints = new ArrayList();
   public static final double // robot's body height
   hBodyZ = 0.36;
   public static final double // body data TO VERIFY
   lBodyX = 0.6, lBodyY = 0.4, lBodyZ = 0.1, mBody = 10.0, IxxBody = 0.1, IyyBody = 0.1, IzzBody = 0.1;
   public static final double // thigh data
   lThighX = 0.04 / 2, lThighY = 0.06 / 3, lThighZ = 0.15, mThigh = 5.63, IxxThigh = inertiaMoment(lThighY, lThighZ),
         IyyThigh = inertiaMoment(lThighX, lThighZ), IzzThigh = inertiaMoment(lThighX, lThighY);
   public static final double // shank data
   lShankX = 0.04 / 2, lShankY = 0.06 / 3, lShankZ = 0.15, mShank = 3.6, IxxShank = inertiaMoment(lShankY, lShankZ), IyyShank = inertiaMoment(lShankX, lShankZ),
         IzzShank = inertiaMoment(lShankX, lShankY);
   public static final double // foot data TO VERIFY
   lFootX = 0.08 / 3, lFootY = 0.06 / 3, lFootZ = 0.1, mFoot = 1.0, IxxFoot = inertiaMoment(lFootY, lFootZ), IyyFoot = inertiaMoment(lFootX, lFootZ),
         IzzFoot = inertiaMoment(lFootX, lFootY);
   public static final double // gearmotor data
   hMotor = 0.12 / 2, rMotor = 0.034 / 2;
   public static final double // robot's data
   lRobot = 0.6, wRobot = 0.36,
         // joint's height
         hThigh = lThighZ + lShankZ + lFootZ, // hip
         hKnee = lShankZ + lFootZ, // knee
         hFoot = lFootZ; // ankle
   Joint rootJoint;

   public GuaraRobot()
   {
      // legs are numbered: 0 front left; 1 hind left; 2 front right; 3 hind right
      super("Guara");
      rootJoint = new FloatingJoint("rootJoint", new Vector3D(0.0, 0.0, 0.0), this);
      ((FloatingJoint) rootJoint).setPosition(0.0, 0.0, hThigh);
      Link bodyLink = body();
      rootJoint.setLink(bodyLink);
      this.addRootJoint(rootJoint);
      bodyLink.addCoordinateSystemToCOM(0.6);
      UniversalJoint abdHip0 = new UniversalJoint("abdHip0X", "abdHip0Y", new Vector3D(lRobot / 2, wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdHip0);
      Link tigh0 = thigh(0);
      abdHip0.setLink(tigh0);
      tigh0.addCoordinateSystemToCOM(0.25);
      PinJoint flexKnee0 = new PinJoint("flexKnee0", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdHip0.addJoint(flexKnee0);
      Link shank0 = shank();
      flexKnee0.setLink(shank0);
      PinJoint ankleFlex0 = new PinJoint("flexAnkle0", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee0.addJoint(ankleFlex0);
      Link foot0 = foot();
      ankleFlex0.setLink(foot0);
      UniversalJoint abdHip1 = new UniversalJoint("abdHip1X", "abdHip1Y", new Vector3D(-lRobot / 2, wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdHip1);
      Link tigh1 = thigh(1);
      abdHip1.setLink(tigh1);
      PinJoint flexKnee1 = new PinJoint("flexKnee1", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdHip1.addJoint(flexKnee1);
      Link shank1 = shank();
      flexKnee1.setLink(shank1);
      PinJoint ankleFlex1 = new PinJoint("ankleFlex1", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee1.addJoint(ankleFlex1);
      Link foot1 = foot();
      ankleFlex1.setLink(foot1);
      UniversalJoint abdHip2 = new UniversalJoint("abdHip2X", "abdHip2Y", new Vector3D(-lRobot / 2, -wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdHip2);
      Link tigh2 = thigh(2);
      abdHip2.setLink(tigh2);
      PinJoint flexKnee2 = new PinJoint("flexKnee2", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdHip2.addJoint(flexKnee2);
      Link shank2 = shank();
      flexKnee2.setLink(shank2);
      PinJoint ankleFlex2 = new PinJoint("ankleFlex2", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee2.addJoint(ankleFlex2);
      Link foot2 = foot();
      ankleFlex2.setLink(foot2);
      UniversalJoint abdHip3 = new UniversalJoint("abdHip3X", "abdHip3Y", new Vector3D(lRobot / 2, -wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdHip3);
      Link tigh3 = thigh(3);
      abdHip3.setLink(tigh3);
      PinJoint flexKnee3 = new PinJoint("flexKnee3", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdHip3.addJoint(flexKnee3);
      Link shank3 = shank();
      flexKnee3.setLink(shank3);
      PinJoint ankleFlex3 = new PinJoint("ankleFlex3", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee3.addJoint(ankleFlex3);
      Link foot3 = foot();
      ankleFlex3.setLink(foot3);
      //Aadd ground contact points
      GroundContactPoint gc0 = new GroundContactPoint("gc0", new Vector3D(0.0, 0.0, -lFootZ), this);
      ankleFlex0.addGroundContactPoint(gc0);
      groundContactPoints.add(gc0);
      GroundContactPoint gc1 = new GroundContactPoint("gc1", new Vector3D(0.0, 0.0, -lFootZ), this);
      groundContactPoints.add(gc1);
      ankleFlex1.addGroundContactPoint(gc1);
      GroundContactPoint gc2 = new GroundContactPoint("gc2", new Vector3D(0.0, 0.0, -lFootZ), this);
      ankleFlex2.addGroundContactPoint(gc2);
      groundContactPoints.add(gc2);
      GroundContactPoint gc3 = new GroundContactPoint("gc3", new Vector3D(0.0, 0.0, -lFootZ), this);
      ankleFlex3.addGroundContactPoint(gc3);
      groundContactPoints.add(gc3);
      LinearGroundContactModel ground = new LinearGroundContactModel(this, this.getRobotsYoVariableRegistry());
      ground.setZStiffness(2000.0);
      ground.setZDamping(1500.0);
      ground.setXYStiffness(50000.0);
      ground.setXYDamping(2000.0);
      ground.setGroundProfile3D(new FlatGroundProfile());
      this.setGroundContactModel(ground);
   }

   /*
    * x axis is red, y axis is white, and z axis is blue.
    */
   private Link body()
   {
      Link ret = new Link("Body");
      ret.setMass(mBody);
      ret.setComOffset(0, 0, 0);
      ret.setMomentOfInertia(IxxBody, IyyBody, IzzBody);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(4 * lBodyX / 5, lBodyY, lBodyZ / 2, YoAppearance.RGBColorFrom8BitInts(180, 76, 0));
      // linkGraphics.addCoordinateSystem(1);
      ret.setLinkGraphics(linkGraphics);
      // roll DOF gearmotor legs 0 and 3 is backward
      linkGraphics.translate(lBodyX / 2 - hMotor, lBodyY / 2 - rMotor / 2, 0.0);
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      linkGraphics.identity();
      linkGraphics.translate(lBodyX / 2 - hMotor, -lBodyY / 2 + rMotor / 2, 0.0);
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      // roll DOF gearmotor legs 1 and 2 is forward
      linkGraphics.identity();
      linkGraphics.translate(-lBodyX / 2, lBodyY / 2 - rMotor / 2, 0.0);
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      linkGraphics.identity();
      linkGraphics.translate(-lBodyX / 2, -lBodyY / 2 + rMotor / 2, 0.0);
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      return ret;
   }

   private Link thigh(int legNumber)
   {
      Link ret = new Link("Thigh");
      ret.setMass(mThigh);
      ret.setComOffset(0, 0, lThighZ / 2);
      ret.setMomentOfInertia(IxxThigh, IyyThigh, IzzThigh);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0, 0, -lThighZ); // coxa
      linkGraphics.addCube(lThighX, lThighY, lThighZ, YoAppearance.DarkSlateGrey());//
      linkGraphics.translate(0, 0, lThighZ); // juntas do quadril
      linkGraphics.rotate(Math.PI / 2, Axis.X);
      linkGraphics.translate(0.0, 0.0, -hMotor / 2);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      // linkGraphics.addCoordinateSystem(0.1);
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link shank()
   {
      Link ret = new Link("Shank");
      ret.setMass(mShank);
      ret.setComOffset(0.0, 0.0, lShankZ / 2);
      ret.setMomentOfInertia(IxxShank, IyyShank, IzzShank);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0, 0, -lShankZ); // shank
      linkGraphics.addCube(lShankX, lShankY, lShankZ, YoAppearance.DarkSlateGrey());
      linkGraphics.translate(0, 0, +lShankZ); // shank
      linkGraphics.rotate(Math.PI / 2, Axis.X);
      linkGraphics.translate(0.0, 0.0, -hMotor / 2);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link foot()
   {
      Link ret = new Link("Foot");
      ret.setMass(mFoot);
      ret.setComOffset(0.0, 0.0, lFootZ / 2);
      ret.setMomentOfInertia(0.0, IyyFoot, 0.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.identity();
      linkGraphics.translate(0, 0, -lFootZ); // feet
      linkGraphics.addCube(lFootX, lFootY, lFootZ, YoAppearance.BlackMetalMaterial());
      linkGraphics.rotate(Math.PI / 2, Axis.X);
      linkGraphics.translate(0.0, lFootZ, 0.0);
      linkGraphics.translate(0.0, 0.0, -hMotor / 2);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   static double inertiaMoment(double sideOne, double sideTwo)
   {
      return (Math.pow(sideOne, 2) + Math.pow(sideTwo, 2));
   }

   public double a2()
   {
      return lThighZ;
   }

   public double a3()
   {
      return lShankZ;
   }

   double a4()
   {
      return lFootZ;
   }

}