import serial
import re
from enum import Enum


class CAEN_DT_Channel(Enum):
    Ch0 = 0
    Ch1 = 1
    Ch2 = 2
    Ch3 = 3


class CAEN_DT_ALARMS(Enum):
    # IS_Unknown      = 0x0000    # OK wasn't returned
    IS_OVC = 0x0008  # At least one ch. in Overcurrent
    IS_OVV = 0x0010  # At least one ch. in OverVoltage
    IS_UNV = 0x0020  # At least one ch. in UnderVoltage
    IS_MAXV = 0x0040  # At least one ch. in MaxV
    IS_TRIP = 0x0080  # At least one ch. in Trip
    IS_MAXPW = 0x0100  # At least one ch. in MaxPower
    IS_TWARN = 0x0200  # At least one ch. in Temp warning (>80°C)
    IS_OVT = 0x0400  # At least one ch. in OVT (>125°C)
    IS_KILL = 0x0800  # At least one ch. Killed
    IS_INTLCK = 0x1000  # At least one ch. in Interlock


class CAEN_DT_IMON_RANGE(Enum):
    Low = "LOW"
    High = "HIGH"


class CAEN_DT_BRDPAR(Enum):
    BDCLR = "BDCLR"  # Clear alarms
    BDILK = "BDILK"  # Grab Interlock Status
    BDNCH = "BDNCH"  # Read Channelcount
    BDNAME = "BDNAME"  # Get Boardtype/-name
    BDSNUM = "BDSNUM"  # Get Serialnumber
    BDFREL = "BDFREL"  # Get Firmware-Release
    BDALARM = "BDALARM"  # Get active alarms
    MACADDR = "MACADDR"  # Get MAC-address
    IPADDR = "IPADDR"  # Get IP-address
    SUBMASK = "SUBMASK"  # Get Submask
    GATEWAY = "GATEWAY"  # Get Gateway-IP
    DHCPEN = "DHCPEN"  # Get DCHP Status


class CAEN_DT_QUERYTYPE(Enum):
    SET = "SET"
    MONITOR = "MON"


class CAEN_DT_QUERYSTATES(Enum):
    Ok = "OK"
    Error = "ERR"


class CAEN_DT_CHNLSTAT(Enum):
    IS_ON = 0x0001  # Channel On
    IS_UP = 0x0002  # Channel Ramping Up
    IS_DOWN = 0x0004  # Channel Ramping Down
    IS_OVC = 0x0008  # Channel in Overcurrent
    IS_OVV = 0x0010  # Channel in OverVoltage
    IS_UNV = 0x0020  # Channel in UnderVoltage
    IS_MAXV = 0x0040  # Channel in MaxV
    IS_TRIP = 0x0080  # Channel in Trip
    IS_MAXPW = 0x0100  # Channel in MaxPower
    IS_TWARN = 0x0200  # Channel in Temperature warning (>80°C)
    IS_OVT = 0x0400  # Channel in OVT (>125°C)
    IS_KILL = 0x0800  # Channel Killed
    IS_INTLCK = 0x1000  # Channel in Interlock


class CAEN_DT_CHNLPAR(Enum):
    ON = "ON"  # Turns channel on
    OFF = "OFF"  # Turns channel off
    VSET = "VSET"  # VSET value
    VSRES = "VSRES"  # Resolution of VSET in Volt
    VSDEC = "VSDEC"  # Decimal digits of VSET
    VMAX = "VMAX"  # Max value of VSET
    VMIN = "VMIN"  # Min value of VSET
    VMON = "VMON"  # VMON value
    VMRES = "VMRES"  # VMON resolution
    VMDEC = "VMDEC"  # Decimal digits of VMON
    ISET = "ISET"  # ISET value
    ISRES = "ISRES"  # Resolution of ISET in μA
    IMAXH = "IMAXH"  # Max value of ISET in high range
    IMAXL = "IMAXL"  # Max value of ISET in low range
    IMIN = "IMIN"  # Min value of ISET
    ISDEC = "ISDEC"  # Decimal digits of ISET
    IMON = "IMON"  # IMON value
    IMRANGE = "IMRANGE"  # IMON range (high /low)
    IMRESL = "IMRESL"  # IMON resolution in low range
    IMRESH = "IMRESH"  # IMON resolution in high range
    IMDECL = "IMDECL"  # Decimal digits of IMON in low range
    IMDECH = "IMDECH"  # Decimal digits of IMON in high range
    MAXV = "MAXV"  # VMAX value
    MVMIN = "MVMIN"  # VMAX minimum value
    MVMAX = "MVMAX"  # VMAX maximum value
    MVRES = "MVRES"  # VMAX resolution
    MVDEC = "MVDEC"  # Decimal digits of VMAX
    PDWN = "PDWN"  # Power down mode Ramp / Kill
    POL = "POL"  # Polarity
    STAT = "STAT"  # Status; Check class CAEN_DT_CHNLSTAT(Enum):
    RUP = "RUP"  # RAMP UP value
    RUPMIN = "RUPMIN"  # RAMP UP minimum value
    RUPMAX = "RUPMAX"  # RAMP UP maximum value
    RUPRES = "RUPRES"  # RAMP UP resolution
    RUPDEC = "RUPDEC"  # Decimal digits of RAMP UP
    RDW = "RDW"  # RAMP DOWN value
    RDWMIN = "RDWMIN"  # RAMP DOWN minimum value
    RDWMAX = "RDWMAX"  # RAMP DOWN maximum value
    RDWRES = "RDWRES"  # RAMP DOWN resolution
    RDWDEC = "RDWDEC"  # Decimal digits of RAMP DOWN
    TRIP = "TRIP"  # Trip value
    TRIPMIN = "TRIPMIN"  # Trip minimum value
    TRIPMAX = "TRIPMAX"  # Trip maximum value
    TRIPRES = "TRIPRES"  # Trip resolution
    TRIPDEC = "TRIPDEC"  # Decimal digits of Trip
    ZCDTC = "ZCDTC"  # Status of ZC Detect; ON = offset current is getting stored; OFF = ready to store offset
    ZCADJ = "ZCADJ"  # Status of ZC Adjust (EN/DIS)


class __CAEN_HVPowerSupplySerialWrapper__:
    def __init__(self, COMPort, Baud):
        # Instanciation of COM-Port
        self.devCOM = serial.Serial()
        # Standard settings of Serial()
        #  port=None,
        #  baudrate=9600,
        #  bytesize=EIGHTBITS,
        #  parity=PARITY_NONE,
        #  stopbits=STOPBITS_ONE,
        #  timeout=None,
        #  xonxoff=False,
        #  rtscts=False,
        #  write_timeout=None,
        #  dsrdtr=False,
        #  inter_byte_timeout=None,
        #  exclusive=None,

        self.devCOM.port = COMPort
        self.devCOM.baudrate = Baud
        self.devCOM.bytesize = 8
        self.devCOM.parity = serial.PARITY_NONE
        self.devCOM.open()
        return

    def __del__(self):
        self.devCOM.close()

    def Send(self, msg):
        self.devCOM.write()
        return

    def Receive(self):
        return self.devCOM.read_until("\r\n")

    def Query(self, msg):
        self.Send(msg)
        return self.Receive()


class CAEN_DT55xxSeries:

    # From Datasheet:
    #  The channel commands have the format: $CMD:***,CH:*,PAR:***,VAL:xxx.xx<CR,LF>
    #  Response string has the format : #CMD:***,VAL:xxx.xx<CR,LF>

    def __init__(self, COMPort, Baudrate):
        self.Device = __CAEN_HVPowerSupplySerialWrapper__(COMPort, Baudrate)

        # Get information which has to be collected once!
        self.__channelCount__ = -1
        stat, strVal = self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_BRDPAR.BDNCH))
        if stat == CAEN_DT_QUERYSTATES.Ok:
            self.__channelCount__ = int(strVal)
        else:
            raise ("Error fetching channel-count")

        self.__boardType__ = "-1"
        stat, strVal = self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_BRDPAR.BDNAME))
        if stat == CAEN_DT_QUERYSTATES.Ok:
            self.__boardType__ = strVal
        else:
            raise ("Error fetching board-name/board-type")

        self.__deviceSN__ = "-1"
        stat, strVal = self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_BRDPAR.BDSNUM))
        if stat == CAEN_DT_QUERYSTATES.Ok:
            self.__deviceSN__ = strVal
        else:
            raise ("Error fetching serial-number")

        self.__fwRel__ = "-1"
        stat, strVal = self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_BRDPAR.BDFREL))
        if stat == CAEN_DT_QUERYSTATES.Ok:
            self.__fwRel__ = strVal
        else:
            raise ("Error fetching firmware-release")

        self.__mac__ = "-1"
        stat, strVal = self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_BRDPAR.MACADDR))
        if stat == CAEN_DT_QUERYSTATES.Ok:
            self.__mac__ = strVal
        else:
            raise ("Error fetching MAC-address")

        self.__ip__ = "-1"
        stat, strVal = self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_BRDPAR.IPADDR))
        if stat == CAEN_DT_QUERYSTATES.Ok:
            self.__ip__ = strVal
        else:
            raise ("Error fetching IP")

        self.__subnetMask__ = "-1"
        stat, strVal = self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_BRDPAR.SUBMASK))
        if stat == CAEN_DT_QUERYSTATES.Ok:
            self.__subnetMask__ = strVal
        else:
            raise ("Error fetching subnet-mask")

        self.__gateway__ = "-1"
        stat, strVal = self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_BRDPAR.GATEWAY))
        if stat == CAEN_DT_QUERYSTATES.Ok:
            self.__gateway__ = strVal
        else:
            raise ("Error fetching gateway")

        self.__dhcpEn__ = "-1"
        stat, strVal = self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_BRDPAR.DHCPEN))
        if stat == CAEN_DT_QUERYSTATES.Ok:
            self.__dhcpEn__ = strVal
        else:
            raise ("Error fetching DHCP-status")

    # Helpers for communication string creation
    def __BuildBrdCmdString__(self, SetOrMon=str, Par=str):
        return "$CMD:" + SetOrMon + ",PAR:" + Par

    def __BuildChnlSetCmdString__(self, Channel=CAEN_DT_Channel, SetOrMon=str, Par=str, StrValue=str):
        return "$CMD:" + SetOrMon + ",CH:" + str(Channel) + ",PAR:" + Par + ",VAL:" + StrValue

    def __BuildChnlMonCmdString__(self, Channel=CAEN_DT_Channel, SetOrMon=str, Par=str):
        return "$CMD:" + SetOrMon + ",CH:" + str(Channel) + ",PAR:" + Par

    # Query-Helper
    def __GetBrdResponseStrgs__(self, msg):
        response = self.Device.Query(msg)
        responseSplit = re.split(response, ", |: ")
        return responseSplit[1], responseSplit[3]

    def __GetMonResponseStrgs__(self, msg):
        return self.__GetBrdResponseStrgs__(msg)

    def __GetSetResponseStr__(self, msg):
        response = self.Device.Query(msg)
        responseSplit = re.split(response, ", |: ")
        return responseSplit[1]  # Should be OK/ERR

    # Board Commands
    def BoardClearAlarm(self):
        return self.__GetSetResponseStr__(self.__BuildBrdCmdString__(CAEN_DT_QUERYTYPE.SET, CAEN_DT_BRDPAR.BDCLR))

    def BoardInterlockStatus(self):
        return self.__GetSetResponseStr__(self.__BuildBrdCmdString__(CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_BRDPAR.BDILK))

    def BoardChannelCount(self):
        return self.__channelCount

    def BoardDeviceType(self):
        return self.__deviceType

    def BoardDeviceSerialNum(self):
        return self.__deviceSN

    def BoardFirmwareReleaseNum(self):
        return self.__fwRel

    def BoardAlarms(self):
        return self.__GetBrdResponseStrgs__(self.__BuildBrdCmdString__(CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_BRDPAR.BDALARM))

    def BoardMAC(self):
        return self.__mac__

    def BoardIP(self):
        return self.__ip__

    def BoardSubnetMask(self):
        return self.__subnetMask__

    def BoardGateway(self):
        return self.__gateway__

    def BoardDHCPState(self):
        return self.__dhcpEn__

    # Channel Set Commands
    def ChnlSetVSET(self, Channel=CAEN_DT_Channel, Voltage=float):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.VSET, "{:4.2f}".format(Voltage))
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetISET(self, Channel=CAEN_DT_Channel, CurrentLimit=float):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.ISET, "{:2.2f}".format(CurrentLimit))
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetIRange(self, Channel=CAEN_DT_Channel, IMRange=CAEN_DT_IMON_RANGE):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.IMRANGE, IMRange)
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetVRampUp(self, Channel=CAEN_DT_Channel, VoltsPerSecond=int):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.RUP, str(VoltsPerSecond))
        # cmd = "$CMD:SET,CH:" + str(Channel) + ",PAR:RUP,VAL:" + str(VoltsPerSecond)
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetVRampDown(self, Channel=CAEN_DT_Channel, VoltsPerSecond=int):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.RDW, str(VoltsPerSecond))
        # cmd = "$CMD:SET,CH:" + str(Channel) + ",PAR:RDW,VAL:" + str(VoltsPerSecond)
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetTripTime(self, Channel=CAEN_DT_Channel, TripSecondsToKill=int):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.TRIP, str(TripSecondsToKill))
        # cmd = "$CMD:SET,CH:" + str(Channel) + ",PAR:TRIP,VAL:" + str(TripSecondsToKill)
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetPwrDownRamped(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.PDWN, "RAMP")
        # cmd = "$CMD:SET,CH:" + str(Channel) + ",PAR:PDWN,VAL:RAMP"
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetPwrDownKillInstant(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.PDWN, "KILL")
        # cmd = "$CMD:SET,CH:" + str(Channel) + ",PAR:PDWN,VAL:KILL"
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetTurnOn(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.ON)  # Special case! (Monitor-method) ON & OFF has no value
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetTurnOff(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.OFF)  # Special case! (Monitor-method) ON & OFF has no value
        return self.__GetSetResponseStr__(cmd)

    def ChnlSetZeroCorrectionEnable(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.ZCDTC, "ON")
        return self.__GetSetStrgFromBoard__(cmd)

    def ChnlSetZeroCorrectionDisable(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.ZCDTC, "OFF")
        return self.__GetSetStrgFromBoard__(cmd)

    def ChnlSetZeroAdjustmentEnable(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.ZCADJ, "EN")
        return self.__GetSetStrgFromBoard__(cmd)

    def ChnlSetZeroAdjustmentDisable(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlSetCmdString__(Channel, CAEN_DT_QUERYTYPE.SET, CAEN_DT_CHNLPAR.ZCADJ, "DIS")
        return self.__GetSetStrgFromBoard__(cmd)

    # Channel Monitoring Methods
    def ChnlMonVSET(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.VSET)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return float(value)

    def ChnlMonVSETRes(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.VSRES)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return float(value)

    def ChnlMonVSETDecDigits(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.VSDEC)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonVMAX(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.VMAX)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonVMIN(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.VMIN)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonVMON(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.VMON)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonVMRes(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.VMRES)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonVMONDecDigits(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.VMDEC)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonISET(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.ISET)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonISRES(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.ISRES)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonIMAXH(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.IMAXH)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonIMAXL(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.IMAXL)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonIMIN(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.IMIN)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonISDEC(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.ISDEC)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonIMON(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.IMON)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonIMRANGE(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.IMRANGE)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonIMRESL(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.IMRESL)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonIMRESH(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.IMRESH)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonIMDECL(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.IMDECL)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonIMDECH(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.IMDECH)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonMAXV(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.MAXV)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonMVMIN(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.MVMIN)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonMVMAX(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.MVMAX)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonMVRES(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.MVRES)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonMVDEC(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.MVDEC)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonPDWN(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.PDWN)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonPOL(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.POL)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonSTAT(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.STAT)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRUP(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RUP)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRUPMIN(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RUPMIN)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRUPMAX(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RUPMAX)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRUPRES(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RUPRES)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRUPDEC(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RUPDEC)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRDW(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RDW)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRDWMIN(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RDWMIN)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRDWMAX(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RDWMAX)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRDWRES(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RDWRES)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonDEC(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RDWDEC)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonTRIP(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.TRIP)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonTRIPMIN(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.TRIPMIN)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonRDWMAX(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.RDWMAX)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonTRIPRES(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.TRIPRES)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonTRIPDEC(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.TRIPDEC)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonZCDTC(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.ZCDTC)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)

    def ChnlMonZCADJ(self, Channel=CAEN_DT_Channel):
        cmd = self.__BuildChnlMonCmdString__(Channel, CAEN_DT_QUERYTYPE.MONITOR, CAEN_DT_CHNLPAR.ZCADJ)
        state, value = self.__GetMonResponseStrgs__(cmd)
        return int(value)


dt55 = CAEN_DT55xxSeries("COM1", 115200)


print("wait here")
