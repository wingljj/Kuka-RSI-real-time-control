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

  Format: RsiContext SchemaVersion 2.0.0 (RSI 5.0+, .rsix single file).
  RSI_CREATE("PoseTrack", ...) appends .rsix itself and finds this file.

  Topology:
      RSI.Out0 -> Limit_X -> POSCORR1.CorrX
      RSI.Out1 -> Limit_Y -> POSCORR1.CorrY
      RSI.Out2 -> Limit_Z -> POSCORR1.CorrZ
      RSI.Out3 -> Limit_A -> POSCORR1.CorrA
      RSI.Out4 -> Limit_B -> POSCORR1.CorrB
      RSI.Out5 -> Limit_C -> POSCORR1.CorrC
      POSCORRMON1 (no inputs; watches total translation / rotation and stops)

  The six Limit objects are safety layer 3. They execute on the controller, so
  they hold even if the host crashes, sends garbage, or is modified.

  ParamId and InIdx/OutIdx are all 0-based, taken from rsiToolbox.xml Index and
  cross-checked against the RSIVisual-generated KR_C5 instance in
  kuka_rsi_driver. Note AxisCorr UpperLimA1 is ParamId=6 in this format but
  ParamID=13 in the legacy one: legacy numbering reserves 7-12 for external
  axes, which the new format splits out into AxisCorrExt.

  Five limit layers, each strictly larger than the one inside it - otherwise the
  inner layer never fires:
      1 host per-cycle increment  kp * error, clamped to vmax * cycle
      2 host accumulated travel   rsi_config.json (currently opened to 1000 mm)
      3 Limit objects below       +/-35 mm, +/-35 deg
      4 POSCORR limits            +/-40 mm, 40 deg
      5 POSCORRMON                45 mm, 45 deg
  POSCORR ships at +/-5 mm / 5 deg, so deploying its defaults would make RSI
  refuse at 5 mm and the host layer never fire at all.
-->
<RsiContext xsi:schemaLocation="RsiContext RsiContext.xsd" SchemaVersion="2.0.0" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="RsiContext">
   <RsiObjects xmlns="RsiDriver">

      <RsiObject ObjTypeId="64" ObjType="Ethernet" ObjId="RSI">
         <Parameters>
            <Parameter IsRuntime="false" Name="ConfigFile" ParamId="0" ParamValue="PoseTrack_ethernet.xml" />
            <!-- Timeout is how many consecutive IPOC mismatches the KRC
                 tolerates; at 12 ms that is about 1.2 s. The host
                 session_gap_ms must be comfortably larger (currently 2000 ms),
                 or the host will move its safety anchor while the KRC still
                 considers the session unbroken. -->
            <Parameter Name="Timeout" ParamId="0" ParamValue="100" />
            <Parameter Name="Flag" ParamId="3" ParamValue="1" />
            <!-- Precision=4 and the host buildSen 'f',4 are a mutual contract. -->
            <Parameter Name="Precision" ParamId="7" ParamValue="4" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_X">
         <Inputs>
            <Input InIdx="0" OutObjId="RSI" OutIdx="0" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-35" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="35" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_Y">
         <Inputs>
            <Input InIdx="0" OutObjId="RSI" OutIdx="1" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-35" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="35" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_Z">
         <Inputs>
            <Input InIdx="0" OutObjId="RSI" OutIdx="2" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-35" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="35" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_A">
         <Inputs>
            <Input InIdx="0" OutObjId="RSI" OutIdx="3" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-35" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="35" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_B">
         <Inputs>
            <Input InIdx="0" OutObjId="RSI" OutIdx="4" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-35" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="35" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="39" ObjType="Limit" ObjId="Limit_C">
         <Inputs>
            <Input InIdx="0" OutObjId="RSI" OutIdx="5" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimit" ParamId="0" ParamValue="-35" />
            <Parameter Name="UpperLimit" ParamId="1" ParamValue="35" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="27" ObjType="PosCorr" ObjId="POSCORR1">
         <Inputs>
            <!-- InIdx 0..5 are CorrX CorrY CorrZ CorrA CorrB CorrC in order -->
            <Input InIdx="0" OutObjId="Limit_X" OutIdx="0" />
            <Input InIdx="1" OutObjId="Limit_Y" OutIdx="0" />
            <Input InIdx="2" OutObjId="Limit_Z" OutIdx="0" />
            <Input InIdx="3" OutObjId="Limit_A" OutIdx="0" />
            <Input InIdx="4" OutObjId="Limit_B" OutIdx="0" />
            <Input InIdx="5" OutObjId="Limit_C" OutIdx="0" />
         </Inputs>
         <Parameters>
            <Parameter Name="LowerLimX" ParamId="0" ParamValue="-40" />
            <Parameter Name="LowerLimY" ParamId="1" ParamValue="-40" />
            <Parameter Name="LowerLimZ" ParamId="2" ParamValue="-40" />
            <Parameter Name="UpperLimX" ParamId="3" ParamValue="40" />
            <Parameter Name="UpperLimY" ParamId="4" ParamValue="40" />
            <Parameter Name="UpperLimZ" ParamId="5" ParamValue="40" />
            <!-- MaxRotAngle is a SINGLE total-angle limit, not one per axis. -->
            <Parameter Name="MaxRotAngle" ParamId="6" ParamValue="40" />
            <!-- RefCorrSys is a TrafoCosysType enum serialised as a NUMBER:
                 World=0  Base=1  RobRoot=2  Tool=3  TTS=4
                 We want BASE, hence 1. The string "Base" is rejected.

                 No IsRuntime attribute: in the whole RSI 5.0 reference file
                 (kuka_rsi_driver KR_C5, RSIVisual-generated) exactly one
                 parameter carries it - Ethernet's ConfigFile. Every enum
                 parameter there (GearTorque TorqueSource, Status Type) is bare
                 Name/ParamId/ParamValue with a numeric value. -->
            <Parameter Name="RefCorrSys" ParamId="0" ParamValue="1" />
         </Parameters>
      </RsiObject>

      <RsiObject ObjTypeId="81" ObjType="PosCorrMon" ObjId="PosCorrMon_1">
         <Parameters>
            <!-- MaxTrans is total translation (not per-axis), i.e. a Euclidean
                 norm. That is why the host layer 2 uses hypot: the two layers
                 only compare if they measure the same quantity.

                 No IsRuntime attribute here. The blueprint marks these
                 CanBeSetAtRuntime="false", but that describes the parameter -
                 it is not something the instance file declares. RSI 5.0 rejects
                 the object with "general error: PosCorrMon_1: MaxTrans" if the
                 attribute is present. The RSIVisual-generated AxisCorrMon in
                 kuka_rsi_driver (RSI 5.0 B485) carries no IsRuntime either. -->
            <Parameter Name="MaxTrans" ParamId="0" ParamValue="45" />
            <Parameter Name="MaxRotAngle" ParamId="1" ParamValue="45" />
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
