within ;
package package_HandOut
  model HelloWorld
    parameter Real a=-7;
    Real x;

  initial equation
    x=1;

  equation
    der(x)-a*x=0;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(__Dymola_Algorithm="Dassl"));
  end HelloWorld;

  model _1D_HandedOut

    constant Real g=Modelica.Constants.g_n;

    //Driver and DriverInterpretor:
    Modelica.Units.SI.DimensionlessRatio APed;

    //VehCtrl:
    Modelica.Units.SI.Force F_xReq;
    Modelica.Units.SI.Torque T_PmReq1;

    //PrimeMover:
    parameter Modelica.Units.SI.Torque T_PmMax=200;
    parameter Modelica.Units.SI.Power P_PmMax=20e3;
    parameter Modelica.Units.SI.Inertia J_Pm=0.25;
    Modelica.Units.SI.Torque T_PmReq2, T_PmInternal, T_Pm;
    Modelica.Units.SI.AngularVelocity w_Pm;

    //Gear:
    parameter Modelica.Units.SI.DimensionlessRatio Ratio=10;
    Modelica.Units.SI.Torque T_WhlRot;
    Modelica.Units.SI.AngularVelocity w_WhlRot;

    //Wheel and Tyre:
    parameter Modelica.Units.SI.Length Radius=0.3;
    parameter Modelica.Units.SI.Inertia J_Whl=0.5;
    parameter Modelica.Units.SI.DimensionlessRatio CC_x=20, mu_Peak=0.6;
    Modelica.Units.SI.Force F_xWhl, F_zWhl;
    Modelica.Units.SI.Velocity v_xWhl;
    Modelica.Units.SI.DimensionlessRatio s_x;

    //Vehicle:
    parameter Modelica.Units.SI.Mass m=1500;
    Modelica.Units.SI.Velocity v_x;

    //Model variant control:
    parameter Boolean ModelTyreSlip=false; // =true; //
    parameter Boolean ModelWheelInertia=false; // =true; //
    parameter Boolean ModelTyreFrictionSaturation=false; // =true; //
    parameter Modelica.Units.SI.Velocity v_Eps=0.001;

  initial equation
    v_x = 0;

  equation

    //Driver and DriverInterpretor (or LongCtrl):
    APed=if time<1 then 0 else 1;
    F_xReq=APed*m*g;

    //VehCtrl:
    T_PmReq1=F_xReq*Radius/Ratio;

    //PrimeMover:
    if w_Pm < P_PmMax/T_PmMax then
      T_PmReq2 = min(T_PmReq1,T_PmMax);
    else
      T_PmReq2 = min(T_PmReq1,P_PmMax/w_Pm);
    end if;
    T_PmInternal=T_PmReq2; //No time delay in torque generation
    J_Pm*der(w_Pm)=T_PmInternal-T_Pm;

    //Gear:
    T_WhlRot=T_Pm*Ratio;
    w_WhlRot=w_Pm/Ratio;

    //Wheel and Tyre:
    if not ModelWheelInertia then
      F_xWhl = T_WhlRot/Radius;
    else
      J_Whl*der(w_WhlRot) = T_WhlRot - F_xWhl*Radius;
    end if;
    if not ModelTyreSlip then
      v_xWhl=w_WhlRot*Radius;
      s_x=0;
    elseif not ModelTyreFrictionSaturation then
      F_xWhl=CC_x*F_zWhl*s_x;
      s_x=(Radius*w_WhlRot-v_xWhl)/max(abs(Radius*w_WhlRot),v_Eps);
    else
      F_xWhl=sign(s_x)*min(CC_x*F_zWhl*abs(s_x), (mu_Peak-0.1*(abs(s_x)-mu_Peak/CC_x))*F_zWhl);
      s_x=min(Radius*w_WhlRot-v_xWhl)/max(abs(Radius*w_WhlRot),v_Eps);
    end if;

    //Vehicle Body:
    v_x=v_xWhl;
    m*der(v_x)=F_xWhl;
    m*g=F_zWhl;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=5, __Dymola_Algorithm="Dassl"));
  end _1D_HandedOut;

  model compare_1_5D_HandedOut

    package_HandOut._1D_HandedOut refVeh
      annotation (Placement(transformation(extent={{-30,8},{-2,22}})));
    package_HandOut._1D_HandedOut lowGearVeh(Ratio=8, CC_x=10) "vehicle with lower gear ratio"
      annotation (Placement(transformation(extent={{8,-20},{36,-6}})));
    parameter Modelica.Units.SI.Torque T_PmMax=300;
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=10, __Dymola_Algorithm="Dassl"));
  end compare_1_5D_HandedOut;
  annotation (uses(Modelica(version="4.0.0")));
end package_HandOut;
