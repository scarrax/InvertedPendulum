model InvertedPendulumMB
  parameter Real m_cart = 5   "Wagenmasse";
  parameter Real m_pend = 0.5 "Pendelmasse";
  parameter Real l = 0.5 "Pendellänge";
  parameter Real d_cart = 0.15;
  parameter Real d_pend = 0.01;

  parameter Real s0 = 0;
  parameter Real v0 = 0;
  parameter Real phi0 = 0.75 * Modelica.Constants.pi / 2;
  parameter Real vphi0 = 0;
  
  input Real tau;
  output Real s = prismatic.s;
  output Real v = prismatic.v;
  output Real phi = revolute.phi;
  output Real vphi = revolute.w;
  
  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(transformation(origin = {-88, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(useAxisFlange = true)  annotation(
    Placement(transformation(origin = {-40, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body bodyWagen(m = m_cart, r_CM = {0, 0, 0})  annotation(
    Placement(transformation(origin = {0, -76}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.Body bodyPendelmasse(m = m_pend, r_CM = {0, 0, 0})  annotation(
    Placement(transformation(origin = {36, 50}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(useAxisFlange = true)  annotation(
    Placement(transformation(origin = {36, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, -l, 0}) annotation(
    Placement(transformation(origin = {36, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Force forceCart annotation(
    Placement(transformation(origin = {-76, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Components.Damper damperCart(d = d_cart) annotation(
    Placement(transformation(origin = {-42, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.Rotational.Components.Damper damperPend(d = d_pend) annotation(
    Placement(transformation(origin = {4, -32}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  //Modelica.Mechanics.Rotational.Components.Fixed fixedFlangeRot annotation(
  //  Placement(transformation(origin = {78, -82}, extent = {{-10, -10}, {10, 10}})));
  //Modelica.Mechanics.Translational.Components.Fixed fixedFlangeTrans annotation(
  //  Placement(transformation(origin = {-60, -82}, extent = {{-10, -10}, {10, 10}})));

initial equation
  //prismatic.s = 0;
  //prismatic.v = 0;
  //revolute.phi = 0.75 * Modelica.Constants.pi / 2;
  //revolute.w = 0;
  prismatic.s = s0;
  prismatic.v = v0;
  revolute.phi = phi0;
  revolute.w = vphi0;

equation
  forceCart.f = tau;

  connect(world.frame_b, prismatic.frame_a) annotation(
    Line(points = {{-78, -60}, {-50, -60}}, color = {95, 95, 95}));
  connect(prismatic.frame_b, revolute.frame_a) annotation(
    Line(points = {{-30, -60}, {36, -60}, {36, -32}}, color = {95, 95, 95}));
  connect(revolute.frame_b, fixedTranslation.frame_a) annotation(
    Line(points = {{36, -12}, {36, -6}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, bodyPendelmasse.frame_a) annotation(
    Line(points = {{36, 14}, {36, 40}}, color = {95, 95, 95}));
  connect(prismatic.frame_b, bodyWagen.frame_a) annotation(
    Line(points = {{-30, -60}, {0, -60}, {0, -66}}, color = {95, 95, 95}));
  connect(damperCart.flange_b, prismatic.support) annotation(
    Line(points = {{-52, -18}, {-52, -46}, {-44, -46}, {-44, -54}}, color = {0, 127, 0}));
  connect(damperCart.flange_a, prismatic.axis) annotation(
    Line(points = {{-32, -18}, {-32, -54}}, color = {0, 127, 0}));
  connect(damperPend.flange_a, revolute.axis) annotation(
    Line(points = {{4, -22}, {26, -22}}));
  connect(damperPend.flange_b, revolute.support) annotation(
    Line(points = {{4, -42}, {26, -42}, {26, -28}}));
  //connect(fixedFlangeRot.flange, revolute.support) annotation(
  //  Line(points = {{78, -82}, {26, -82}, {26, -28}}));
  //connect(fixedFlangeTrans.flange, prismatic.support) annotation(
  //  Line(points = {{-60, -82}, {-60, -54}, {-44, -54}}, color = {0, 127, 0}));
  connect(forceCart.flange, prismatic.axis) annotation(
    Line(points = {{-66, -30}, {-32, -30}, {-32, -54}}, color = {0, 127, 0}));

annotation(
    uses(Modelica(version = "4.0.0")));
end InvertedPendulumMB;
