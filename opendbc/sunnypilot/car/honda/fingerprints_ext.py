"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car.structs import CarParams
from opendbc.car.honda.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS_EXT = {
  CAR.HONDA_ACCORD: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TVA,A150\x00\x00',
    ],
  },
  CAR.HONDA_CIVIC: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TBA,A030\x00\x00',
    ],
  },
  CAR.HONDA_CIVIC_BOSCH: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TGG,A020\x00\x00',
      b'39990-TGG,A120\x00\x00',
    ],
  },
  CAR.HONDA_CRV_5G: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TLA,A040\x00\x00',
    ],
  },
  CAR.HONDA_CLARITY: {
    (Ecu.shiftByWire, 0x18da0bf1, None): [
      b'54008-TRW-A910\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TRW-A010\x00\x00',
      b'57114-TRW-A020\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TRW-A020\x00\x00',
      b'39990-TRW,A020\x00\x00',  # modified firmware
      b'39990,TRW,A020\x00\x00',  # extra modified firmware
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TRW-A210\x00\x00',
      b'77959-TRW-A220\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TRW-A010\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36161-TRW-A110\x00\x00',
    ],
  },
  CAR.HONDA_CITY_7G: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-T14-B510\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-T14-M210\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-T14-B810\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'8S102-T14-P020\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-T14-M510\x00\x00',
    ],
    (Ecu.transmission, 0x18da1ef1, None): [
      b'28101-63B-M510\x00\x00',
    ],
  },
}

# CAN fingerprints (buses 0 and 1), for platforms without known FW versions.
# ACURA_MDX_3G_SERIAL: from a 2018 Acura MDX with an LKAS/CAN adapter board. 520/521 (and 399/427/522 while openpilot
# is steering) are sent by the adapter board on the camera side and reach bus 0 while the harness relay is closed.
FINGERPRINTS_EXT = {
  CAR.ACURA_MDX_3G_SERIAL: [{
    57: 3, 145: 8, 308: 5, 316: 8, 334: 8, 342: 6, 344: 8, 379: 8, 380: 8, 398: 3, 399: 4, 411: 5, 419: 8, 420: 8, 422: 8, 425: 8, 426: 8,
    427: 3, 432: 7, 450: 8, 463: 8, 464: 8, 476: 4, 487: 4, 490: 8, 506: 8, 520: 8, 521: 8, 522: 8, 542: 7, 545: 5, 546: 3, 597: 8, 660: 8,
    773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 825: 4, 829: 5, 837: 5, 856: 7, 871: 8, 881: 8, 882: 2, 884: 7, 891: 8,
    892: 8, 918: 7, 923: 2, 927: 8, 929: 8, 983: 8, 985: 3, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1070: 8, 1080: 8, 1081: 8, 1088: 8,
    1089: 8, 1090: 8, 1092: 1, 1108: 8, 1110: 8, 1125: 8, 1137: 8, 1296: 8, 1348: 5, 1600: 5, 1601: 8, 1612: 5, 1614: 5, 1615: 8, 1618: 5,
    1665: 5
  }],
}
