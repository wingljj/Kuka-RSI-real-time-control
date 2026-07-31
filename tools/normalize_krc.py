"""把 krc/ 下三个部署文件规范化为 KRL/RSI 能正确读取的形式。

KRL 编辑器与 RSI 解析器按单字节代码页读文件，UTF-8 的中文注释在示教器上
显示为乱码。KUKA 官方生成的同类文件是纯 ASCII + CRLF，这里对齐。

用法：python tools/normalize_krc.py
"""
import io
import os

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

ETH = '''<ROOT>
   <CONFIG>
      <!-- IP_NUMBER is the HOST address, not the robot. RSI dials out from the
           KRC side. Must match listen_ip / listen_port in
           config/rsi_config.json and the GUI listen-address field. -->
      <IP_NUMBER>192.168.44.1</IP_NUMBER>
      <PORT>59152</PORT>
      <!-- SENTYPE must equal the string the host puts in Sen Type, i.e.
           sen_type in rsi_config.json. If they differ the KRC silently discards
           every reply while the host sees nothing wrong - it believes it
           answered. That failure ends in an RSI timeout stop with the host still
           reporting zero lost packets. -->
      <SENTYPE>ImFree</SENTYPE>
      <ONLYSEND>FALSE</ONLYSEND>
   </CONFIG>

   <!-- TYPE   = BOOL | STRING | LONG | DOUBLE
        INDX   = INTERNAL  filled in by RSI itself (everything DEF_*)
        INDX   = n         maps to channel n of the Ethernet object, 1-based
        HOLDON = 1         hold the previous value when no new data arrives -->

   <SEND>
      <ELEMENTS>
         <!-- Cartesian actual pose. The host closed loop and its safety ledger
              both key off this; the one element that cannot be omitted. -->
         <ELEMENT TAG="DEF_RIst"  TYPE="DOUBLE" INDX="INTERNAL" />
         <!-- Cartesian setpoint pose. Host-side diagnostics only. -->
         <ELEMENT TAG="DEF_RSol"  TYPE="DOUBLE" INDX="INTERNAL" />
         <!-- Joint actual / setpoint. POSCORR does not need them, but they make
              "cartesian looks fine yet a joint hit its limit" diagnosable. -->
         <ELEMENT TAG="DEF_AIPos" TYPE="DOUBLE" INDX="INTERNAL" />
         <ELEMENT TAG="DEF_ASPos" TYPE="DOUBLE" INDX="INTERNAL" />
         <!-- DEF_Delay is the KRC own count of late / lost packets. It is the
              only way the host can see "the controller thinks I am dropping
              packets": the host counter cannot see a reply that arrived late,
              nor one the KRC discarded over a SENTYPE mismatch. Keep it. -->
         <ELEMENT TAG="DEF_Delay" TYPE="LONG"   INDX="INTERNAL" />
      </ELEMENTS>
   </SEND>

   <RECEIVE>
      <ELEMENTS>
         <!-- The six components of RKorr in the host reply, feeding Ethernet
              output channels 1..6. The object graph then routes each one through
              a Limit object into POSCORR.

              These are PER-CYCLE DISPLACEMENT INCREMENTS, not target
              coordinates. POSCORR runs in RELATIVE mode, so sending an absolute
              coordinate would command the robot to cover that whole distance
              within one interpolation cycle.

              HOLDON is deliberately 0. Under incremental semantics, holding the
              previous value means that if the host stalls the KRC re-applies the
              last increment every cycle: 0.6 mm times the 100 cycles of Timeout
              is 60 mm, and the host ledger would count none of it. HOLDON=1 is
              safe for an absolute interface and the worst possible choice for an
              incremental one. -->
         <ELEMENT TAG="RKorr.X" TYPE="DOUBLE" INDX="1" HOLDON="0" />
         <ELEMENT TAG="RKorr.Y" TYPE="DOUBLE" INDX="2" HOLDON="0" />
         <ELEMENT TAG="RKorr.Z" TYPE="DOUBLE" INDX="3" HOLDON="0" />
         <ELEMENT TAG="RKorr.A" TYPE="DOUBLE" INDX="4" HOLDON="0" />
         <ELEMENT TAG="RKorr.B" TYPE="DOUBLE" INDX="5" HOLDON="0" />
         <ELEMENT TAG="RKorr.C" TYPE="DOUBLE" INDX="6" HOLDON="0" />
      </ELEMENTS>
   </RECEIVE>
</ROOT>
'''

RSIX = '''<?xml version="1.0" encoding="utf-8"?>
<!--
  RSI context - POSCORR pose tracking (BASE frame, RELATIVE increments)

  Format: RsiContext SchemaVersion 2.0.0 (.rsix single file).
  RSI_CREATE("PoseTrack", ...) appends .rsix itself and finds this file.

  PROVENANCE This file is a deliberate near-clone of a known-good PosCorr
  context: ROS_RSI_CONTEXT.rsix (RSIVisual iiQWorks.AppBuilder). Every
  attribute style here - IsPublic on the PosCorr limits, IsRuntime on
  RefCorrSys and both PosCorrMon parameters, SignalName on every Input, the
  ObjId naming - is copied from that file rather than derived. Two earlier
  attempts failed RSI_CREATE because the attributes were reasoned out from
  blueprints instead of copied from a file known to load.

  Differences from that reference, and only these:
    - ConfigFile points at PoseTrack_ethernet.xml
    - no Stop object (PoseTrack_ethernet.xml declares 6 receive channels, so
      there is no Out7 for a Stop object to read)

  Topology:
      ETHERNET1.Out0 -> Limit_X -> PosCorr_1.CorrX
      ETHERNET1.Out1 -> Limit_Y -> PosCorr_1.CorrY
      ETHERNET1.Out2 -> Limit_Z -> PosCorr_1.CorrZ
      ETHERNET1.Out3 -> Limit_A -> PosCorr_1.CorrA
      ETHERNET1.Out4 -> Limit_B -> PosCorr_1.CorrB
      ETHERNET1.Out5 -> Limit_C -> PosCorr_1.CorrC
      PosCorrMon_1 (no inputs; watches total translation / rotation, stops)

  ParamId is the 0-based sequential position of the parameter within its own
  type space for that object, verified empirically: the RSI 5.0 AxisCorr in
  kuka_rsi_driver has UpperLimA1 at ParamId=6 while rsiToolbox.xml lists it at
  Index=13, because legacy numbering reserves 13-18 for the upper limits and
  7-12 for external axes. POSCORR's toolbox indices are contiguous 1..14, so
  ParamId = Index - 1. RefCorrSys is an EnumParameter and therefore has its own
  index space, which is why it is ParamId=0 alongside LowerLimX at ParamId=0.

  LIMIT VALUES These are the reference's own proven values, on purpose. Getting
  RSI_CREATE to succeed once matters more than the final envelope, and raising
  them afterwards is a monotone experiment instead of an N-way guess.

  Note what each layer actually measures - they are not the same quantity:
      Limit objects   clamp the PER-CYCLE increment          +/-5 mm, +/-5 deg
      PosCorr         clamps the ACCUMULATED correction      +/-5 mm, 5 deg
      PosCorrMon      stops on accumulated total             6 mm, 6 deg
  The host per-cycle increment is at most kp * vmax * cycle = 0.012 mm, so the
  Limit objects never bind in normal operation; they exist to bound a garbage
  frame. PosCorr is the real ceiling: the robot will hard-stop once total
  correction reaches 5 mm. Raise LowerLim/UpperLim/MaxRotAngle here, and
  PosCorrMon above them, after the first successful run.
-->
<RsiContext xsi:schemaLocation="RsiContext RsiContext.xsd" SchemaVersion="2.0.0" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="RsiContext">
   <RsiObjects xmlns="RsiDriver">

      <RsiObject ObjTypeId="64" ObjType="Ethernet" ObjId="ETHERNET1">
         <Parameters>
            <Parameter IsRuntime="false" Name="ConfigFile" ParamId="0" ParamValue="PoseTrack_ethernet.xml" />
            <!-- Timeout is how many consecutive missed cycles the KRC tolerates;
                 at 12 ms that is about 1.2 s. The host session_gap_ms must stay
                 comfortably larger (currently 2000 ms), or the host would move
                 its safety anchor while the KRC still considers the session
                 unbroken. -->
            <Parameter Name="Timeout" ParamId="0" ParamValue="100" />
            <Parameter Name="Flag" ParamId="3" ParamValue="1" />
            <!-- Precision=4 and the host buildSen 'f',4 are a mutual contract. -->
            <Parameter Name="Precision" ParamId="7" ParamValue="4" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_X">
         <Inputs>
            <Input InIdx="0" OutObjId="ETHERNET1" OutIdx="0" SignalName="ETHERNET1|Out1" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-5" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="5" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_Y">
         <Inputs>
            <Input InIdx="0" OutObjId="ETHERNET1" OutIdx="1" SignalName="ETHERNET1|Out2" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-5" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="5" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_Z">
         <Inputs>
            <Input InIdx="0" OutObjId="ETHERNET1" OutIdx="2" SignalName="ETHERNET1|Out3" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-5" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="5" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_A">
         <Inputs>
            <Input InIdx="0" OutObjId="ETHERNET1" OutIdx="3" SignalName="ETHERNET1|Out4" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-5" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="5" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_B">
         <Inputs>
            <Input InIdx="0" OutObjId="ETHERNET1" OutIdx="4" SignalName="ETHERNET1|Out5" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-5" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="5" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_C">
         <Inputs>
            <Input InIdx="0" OutObjId="ETHERNET1" OutIdx="5" SignalName="ETHERNET1|Out6" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-5" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="5" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="27" ObjType="PosCorr" ObjId="PosCorr_1">
         <Inputs>
            <!-- InIdx 0..5 are CorrX CorrY CorrZ CorrA CorrB CorrC, in the order
                 the RSIInPorts are declared in rsiToolbox.xml. -->
            <Input InIdx="0" OutObjId="Limit_X" OutIdx="0" SignalName="Limit_X|Out1" />
            <Input InIdx="1" OutObjId="Limit_Y" OutIdx="0" SignalName="Limit_Y|Out1" />
            <Input InIdx="2" OutObjId="Limit_Z" OutIdx="0" SignalName="Limit_Z|Out1" />
            <Input InIdx="3" OutObjId="Limit_A" OutIdx="0" SignalName="Limit_A|Out1" />
            <Input InIdx="4" OutObjId="Limit_B" OutIdx="0" SignalName="Limit_B|Out1" />
            <Input InIdx="5" OutObjId="Limit_C" OutIdx="0" SignalName="Limit_C|Out1" />
         </Inputs>
         <Parameters>
            <Parameter IsPublic="true" Name="LowerLimX" ParamId="0" ParamValue="-5" />
            <Parameter IsPublic="true" Name="LowerLimY" ParamId="1" ParamValue="-5" />
            <Parameter IsPublic="true" Name="LowerLimZ" ParamId="2" ParamValue="-5" />
            <Parameter IsPublic="true" Name="UpperLimX" ParamId="3" ParamValue="5" />
            <Parameter IsPublic="true" Name="UpperLimY" ParamId="4" ParamValue="5" />
            <Parameter IsPublic="true" Name="UpperLimZ" ParamId="5" ParamValue="5" />
            <!-- MaxRotAngle is a SINGLE total-angle limit, not one per axis. -->
            <Parameter IsPublic="true" Name="MaxRotAngle" ParamId="6" ParamValue="5" />
            <!-- RefCorrSys is an EnumParameter (EnumType TrafoCosysType) that
                 serialises as a NUMBER, not the string "Base". 1 = Base, which
                 is what the reference file writes for a BASE-frame correction.
                 The blueprint marks it CanBeSetAtRuntime="false", and the
                 corresponding instance attribute is IsRuntime="false". -->
            <Parameter IsRuntime="false" Name="RefCorrSys" ParamId="0" ParamValue="1" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="81" ObjType="PosCorrMon" ObjId="PosCorrMon_1">
         <Parameters>
            <!-- MaxTrans is total translation - a single Euclidean norm, not a
                 per-axis figure. That is why the host safety ledger uses hypot:
                 two layers can only be compared if they measure the same thing.
                 Rotation stays per-axis on the host, since Euler angles have no
                 meaningful Euclidean norm. -->
            <Parameter IsRuntime="false" Name="MaxTrans" ParamId="0" ParamValue="6" />
            <Parameter IsRuntime="false" Name="MaxRotAngle" ParamId="1" ParamValue="6" />
         </Parameters>
      </RsiObject>

   </RsiObjects>
</RsiContext>
'''


def write_ascii_crlf(path, text, bom=False):
    data = text.replace("\r\n", "\n").replace("\n", "\r\n").encode("ascii")
    if bom:
        data = b"\xef\xbb\xbf" + data
    with open(path, "wb") as f:
        f.write(data)
    print("wrote %-34s %6d bytes  BOM=%s" % (path, len(data), bom))


write_ascii_crlf("krc/PoseTrack_ethernet.xml", ETH, bom=False)
# 官方生成的 .rsix 带 BOM，保持一致
write_ascii_crlf("krc/PoseTrack.rsix", RSIX, bom=True)

# .src 已是 ASCII，只需确保 CRLF
with io.open("krc/PoseTrack.src", encoding="ascii") as f:
    src = f.read()
write_ascii_crlf("krc/PoseTrack.src", src, bom=False)
