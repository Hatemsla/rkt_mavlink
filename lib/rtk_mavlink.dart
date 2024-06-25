import 'dart:ffi';
import 'dart:io';
import 'dart:isolate';

import 'package:dbus/dbus.dart';
import 'package:flutter/material.dart';
import 'package:rtk_mavlink/mavlink_classes.dart';
import 'package:rtk_mavlink/models.dart';

import 'rtk_mavlink_bindings_generated.dart';

const String _libName = 'rtk_mavlink';

bool isHeartbeatAlreadyRecived = false;
int customSeq = 0;

// Запрос данных о положении БПЛА
(Array<Uint8>, int) requestAttitude() {
  try {
    var attitudePointer = _bindings.request_attitude();

    debugPrint(attitudePointer.tx_msg_len.toString());

    return (attitudePointer.tx_msg_buffer, attitudePointer.tx_msg_len);
  } catch (e, st) {
    // debugPrint(e.toString());
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

// Запрос данных о системном статусе
(Array<Uint8>, int) requestSysStatus() {
  try {
    var sysStatusPointer = _bindings.request_sys_status();

    return (sysStatusPointer.tx_msg_buffer, sysStatusPointer.tx_msg_len);
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

// Запрос данных о навигации
(Array<Uint8>, int) requestGpsStatus() {
  try {
    var gpsStatusPointer = _bindings.request_gps_status();

    return (gpsStatusPointer.tx_msg_buffer, gpsStatusPointer.tx_msg_len);
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

// Запрос данных о положении на местности
(Array<Uint8>, int) requestGlobalPositionInt() {
  try {
    var globalPositionIntPointer = _bindings.request_global_position_int();

    return (
      globalPositionIntPointer.tx_msg_buffer,
      globalPositionIntPointer.tx_msg_len
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestLocalPositionNed() {
  try {
    var globalPositionIntPointer = _bindings.request_local_position_ned();

    return (
      globalPositionIntPointer.tx_msg_buffer,
      globalPositionIntPointer.tx_msg_len
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestMissionCount(int missionCount) {
  try {
    var missionCountPointer = _bindings.request_mission_count(missionCount);

    return (
      missionCountPointer.tx_msg_buffer,
      missionCountPointer.tx_msg_len,
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestMissionNavWaypoint(
    int seq, int lat, int lon, int alt) {
  try {
    var missionItemIntPointer =
        _bindings.request_mission_nav_waypoint(seq, lat, lon, alt);

    return (
      missionItemIntPointer.tx_msg_buffer,
      missionItemIntPointer.tx_msg_len,
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestMissionNavLand(int seq, int lat, int lon, int alt) {
  try {
    var missionItemIntPointer =
        _bindings.request_mission_nav_land(seq, lat, lon, alt);

    return (
      missionItemIntPointer.tx_msg_buffer,
      missionItemIntPointer.tx_msg_len,
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestMissionNavTakeoff(
    int seq, int lat, int lon, int alt) {
  try {
    var missionItemIntPointer =
        _bindings.request_mission_nav_takeoff(seq, lat, lon, alt);

    return (
      missionItemIntPointer.tx_msg_buffer,
      missionItemIntPointer.tx_msg_len,
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestMissionNavReturnToLaunch(int seq) {
  try {
    var missionItemIntPointer =
        _bindings.request_mission_nav_return_to_launch(seq);

    return (
      missionItemIntPointer.tx_msg_buffer,
      missionItemIntPointer.tx_msg_len,
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestMissionDoSetMode() {
  try {
    var cmd = _bindings.request_mission_do_set_mode();

    return (
      cmd.tx_msg_buffer,
      cmd.tx_msg_len,
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestMissionStart() {
  try {
    var cmd = _bindings.request_mission_start();

    return (
      cmd.tx_msg_buffer,
      cmd.tx_msg_len,
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestCmdArmDisarm(double arm) {
  try {
    var cmd = _bindings.request_cmd_arm_disarm(arm);

    return (
      cmd.tx_msg_buffer,
      cmd.tx_msg_len,
    );
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

MavlinkHeartbeat? mavlinkHeartbeat;
MavlinkSysStatus? mavlinkSystStatus;
MavlinkGpsStatus? mavlinkGpsStatus;
MavlinkAttitude? mavlinkAttitude;
MavlinkGlobalPositionInt? mavlinkGlobalPositionInt;
MavlinkLocalPositionNed? mavlinkLocalPositionNed;
MavlinkMissionRequestInt? mavlinkMissionRequestInt;
MavlinkMissionRequest? mavlinkMissionRequest;
MavlinkMissionAck? mavlinkMissionAck;
MavlinkStatusText? mavlinkStatusText;

List<int> requestCount = [];
List<int> currentSeq = [];
int res = -1;
bool isMissionSent = false;

const int defaultTimeout = 1500;
const domen = 'org.usb.UsbDriver';
const path = '/org/usb/UsbDriver';

Future<void> _sendData(DBusRemoteObject remoteObject, List<int> list) async {
  if (list.isEmpty) return;

  await remoteObject.callMethod(
    domen,
    'SendData',
    [
      DBusArray.int32(list),
    ],
  );
}

List<int> _parseArrayToList((Array<Uint8>, int) array) {
  final List<int> list = [];
  for (var i = 0; i < array.$2; i++) {
    list.add(array.$1[i]);
  }

  return list;
}

Future<void> _sendStartMission(SendPort sendPort) async {
  var client1 = DBusClient.session();
  var remoteObject1 =
      DBusRemoteObject(client1, name: domen, path: DBusObjectPath(path));

  var mode = requestMissionDoSetMode();
  List<int> modeIntList = _parseArrayToList(mode);
  await _sendData(remoteObject1, modeIntList);
  debugPrint("Режим AUTO установлен: $modeIntList");

  var missionStart = requestMissionStart();
  List<int> missionStartIntList = _parseArrayToList(missionStart);
  await _sendData(remoteObject1, missionStartIntList);
  debugPrint("Миссия начата: $missionStartIntList");

  var arm = requestCmdArmDisarm(1);
  List<int> armIntList = _parseArrayToList(arm);
  await _sendData(remoteObject1, armIntList);
  debugPrint("Арм установлен: $armIntList");

  Isolate.current.kill(priority: Isolate.immediate);
}

Future<void> sendMissionCount(DBusClient client1,
    DBusRemoteObject remoteObject1, int missionCount) async {
  var reqMissionCount =
      requestMissionCount(missionCount); // количество миссий -1 будет
  List<int> missionCountIntList = _parseArrayToList(reqMissionCount);

  await _sendData(remoteObject1, missionCountIntList);

  debugPrint("Количество миссий установлено: $missionCountIntList");
}

Future<void> loadMissionElement(int type, int seq, int lat, int lon, int alt,
    DBusClient client1, DBusRemoteObject remoteObject1) async {
  (Array<Uint8>, int) missionItemInt = (const Array<Uint8>(0), 0);
  List<int> missionItemIntList;
  switch (type) {
    case 0:
    case 1:
      missionItemInt = requestMissionNavTakeoff(seq, lat, lon, alt);
      debugPrint(type == 0 ? "Фейк" : "Старт отправлен");
      break;
    case 2:
      missionItemInt = requestMissionNavWaypoint(seq, lat, lon, alt);
      debugPrint("Чекпоинт отправлен");
      break;
    case 3:
      missionItemInt = requestMissionNavReturnToLaunch(seq);
      debugPrint("На базу");
      break;
  }

  missionItemIntList = _parseArrayToList(missionItemInt);
  await _sendData(remoteObject1, missionItemIntList);
  debugPrint("$missionItemIntList");
}

Future<void> _sendMission(List<dynamic> args) async {
  SendPort sendPort = args[0];
  List<WaypointModelRtk> waypoints = args[1];
  final port = ReceivePort();
  sendPort.send(port.sendPort);

  int receivedMissionSeq = -1;
  bool receivedIsMissionRequest = false;
  bool receivedIsMissionAck = false;

  final iSMCountRPort = ReceivePort();
  final iSMElementsRPort = ReceivePort();

  await Isolate.spawn(_sendMissionCount, [
    receivedIsMissionRequest,
    receivedMissionSeq,
    iSMCountRPort.sendPort,
    waypoints.length
  ]);
  await Isolate.spawn(_sendMissionElements, [
    receivedIsMissionAck,
    receivedIsMissionRequest,
    receivedMissionSeq,
    iSMElementsRPort.sendPort,
    waypoints
  ]);

  final iSMCountSendPort = await iSMCountRPort.first as SendPort;
  final iSMElementsSendPort = await iSMElementsRPort.first as SendPort;

  port.listen((data) {
    debugPrint("DATA: $data");
    debugPrint(
        "--------------------------------------------------------------------");
    if (data is List && data.length == 3) {
      receivedMissionSeq = data[0];
      receivedIsMissionRequest = data[1];
      receivedIsMissionAck = data[2];

      iSMCountSendPort.send([receivedIsMissionRequest, receivedMissionSeq]);
      iSMElementsSendPort.send(
          [receivedIsMissionAck, receivedIsMissionRequest, receivedMissionSeq]);
    }
  });
}

Future<void> _sendMissionElements(List<dynamic> args) async {
  var client = DBusClient.session();
  var remoteObject =
      DBusRemoteObject(client, name: domen, path: DBusObjectPath(path));

  bool receivedIsMissionAck = args[0];
  bool receivedIsMissionRequest = args[1];
  int receivedMissionSeq = args[2];
  SendPort sendPort = args[3];
  List<WaypointModelRtk> waypoints = args[4];
  waypoints.insert(0, WaypointModelRtk(latitude: 0, longitude: 0, altitude: 0));
  waypoints.add(WaypointModelRtk(latitude: 0, longitude: 0, altitude: 0));

  final port = ReceivePort();
  sendPort.send(port.sendPort);

  port.listen((message) {
    if (message is List) {
      receivedIsMissionAck = message[0];
      receivedIsMissionRequest = message[1];
      receivedMissionSeq = message[2];
      debugPrint("IsNew: $receivedIsMissionRequest");
    }
  });

  while (!receivedIsMissionAck) {
    if (receivedIsMissionRequest) {
      debugPrint("receivedMissionSeq: $receivedMissionSeq");

      int type;
      if (receivedMissionSeq == 0 || receivedMissionSeq == 1) {
        type = receivedMissionSeq; // 0 for takeoff, 1 for start
      } else if (receivedMissionSeq == waypoints.length - 1) {
        type = 3; // 3 for return to launch
      } else {
        type = 2; // 2 for waypoint
      }
      await loadMissionElement(
        type,
        receivedMissionSeq,
        waypoints[receivedMissionSeq].latitude,
        waypoints[receivedMissionSeq].longitude,
        waypoints[receivedMissionSeq].altitude,
        client,
        remoteObject,
      );

      receivedIsMissionRequest = false;
    }
    await Future.delayed(Duration.zero);
  }
}

Future<void> _sendMissionCount(List<dynamic> args) async {
  var client1 = DBusClient.session();
  var remoteObject1 =
      DBusRemoteObject(client1, name: domen, path: DBusObjectPath(path));
  int currentTimeMills = 0;
  bool receivedIsMissionRequest = args[0];
  int receivedMissionSeq = args[1];
  SendPort sendPort = args[2];
  int missionCount = args[3];

  final port = ReceivePort();
  sendPort.send(port.sendPort);

  port.listen((message) {
    if (message is List) {
      receivedIsMissionRequest = message[0];
      receivedMissionSeq = message[1];
    }
  });

  await sendMissionCount(
      client1, remoteObject1, missionCount + 2); // + set home and rtl
  currentTimeMills = getCurrentTimeMillis();

  while (!receivedIsMissionRequest) {
    if (getCurrentTimeMillis() - currentTimeMills > defaultTimeout) {
      debugPrint("receivedMissionSeq: $receivedMissionSeq");
      await sendMissionCount(client1, remoteObject1, missionCount + 2);
      currentTimeMills = getCurrentTimeMillis();
    }
    await Future.delayed(Duration.zero);
  }
}

late ReceivePort _sendMissionRPort;
late ReceivePort _sendStartMissionRPort;

late SendPort _sendMissionSPort;
late SendPort _sendStartMissionSPort;

int currentTimeMillis = 0;
bool isReceivePort = false;

int getCurrentTimeMillis() {
  return DateTime.now().millisecondsSinceEpoch;
}

Future<void> startSendMissionStatic() async {
  _sendMissionRPort = ReceivePort();
  await Isolate.spawn(_sendMission, [_sendMissionRPort.sendPort, []]);
  _sendMissionSPort = await _sendMissionRPort.first as SendPort;
  isReceivePort = true;
}

Future<void> loadMission(List<WaypointModelRtk> waypoints) async {
  _sendMissionRPort = ReceivePort();
  await Isolate.spawn(_sendMission, [_sendMissionRPort.sendPort, waypoints]);
  _sendMissionSPort = await _sendMissionRPort.first as SendPort;
  isReceivePort = true;
}

Future<void> startMission() async {
  _sendStartMissionRPort = ReceivePort();
  await Isolate.spawn(_sendStartMission, _sendStartMissionRPort.sendPort);
  _sendStartMissionSPort = await _sendStartMissionRPort.first as SendPort;
}

// Функция обновления данных
Future<List<MavlinkMessage>> updateData(List<int> newBytes) async {
  List<MavlinkMessage> messages = [];

  for (var i = 0; i < newBytes.length; i++) {
    // После обновления данных, производится обновление переменных внутри _bindings
    res = _bindings.update_data(newBytes[i]);

    if (res == 40) {
      if (isReceivePort) {
        debugPrint("IS_MISSION_REQUEST: ${_bindings.is_mission_request}");
        debugPrint(
            "--------------------------------------------------------------------");

        bool isMissionAck = false;

        if (_bindings.is_mission_ack == 1 &&
            _bindings.rx_mission_ack.type == 13) {
          isMissionAck = false;
        } else if (_bindings.is_mission_ack == 1) {
          isMissionAck = true;
        }

        _sendMissionSPort.send([
          _bindings.rx_mission_request.seq,
          _bindings.is_mission_request == 1 ? true : false,
          isMissionAck
        ]);

        debugPrint('Request seq: ${_bindings.rx_mission_request.seq}\n'
            'targetComponent: ${_bindings.rx_mission_request.target_component}\n'
            'targetSystem: ${_bindings.rx_mission_request.target_system}\n'
            'missionType: ${_bindings.rx_mission_request.mission_type}');
        debugPrint(
            "--------------------------------------------------------------------");

        if (_bindings.is_mission_request == 1) {
          _bindings.is_mission_request = -1;
        }

        if (_bindings.is_mission_ack == 1) {
          _bindings.is_mission_ack = -1;
        }
      }
    } else if (res == 253) {
      debugPrint(
          'STATUSTEXT: ${convertFfiArrayToString(_bindings.rx_statustext.text, 50)}');
      debugPrint(
          "--------------------------------------------------------------------");
    } else if (res == 47 && isReceivePort) {
      debugPrint('Ack type: ${_bindings.rx_mission_ack.type}\n'
          'Ack targetComponent: ${_bindings.rx_mission_ack.target_component}\n'
          'Ack targetSystem: ${_bindings.rx_mission_ack.target_system}\n'
          'Ack missionType: ${_bindings.rx_mission_ack.mission_type}');
      debugPrint(
          "--------------------------------------------------------------------");

      bool isMissionAck = false;

      if (_bindings.is_mission_ack == 1 &&
          _bindings.rx_mission_ack.type == 13) {
        isMissionAck = false;
      } else if (_bindings.is_mission_ack == 1) {
        isMissionAck = true;
      }

      _sendMissionSPort
          .send([_bindings.rx_mission_request.seq, true, isMissionAck]);
    }

    isHeartbeatAlreadyRecived = _bindings.already_received_heartbeat == 1;
    customSeq = _bindings.custom_seq;

    mavlinkHeartbeat = MavlinkHeartbeat(
      type: _bindings.rx_heartbeat.type,
      autopilot: _bindings.rx_heartbeat.autopilot,
      baseMode: _bindings.rx_heartbeat.base_mode,
      customMode: _bindings.rx_heartbeat.custom_mode,
      systemStatus: _bindings.rx_heartbeat.system_status,
      mavlinkVersion: _bindings.rx_heartbeat.mavlink_version,
    );

    mavlinkSystStatus = MavlinkSysStatus(
      onboardControlSensorsPresent:
          _bindings.rx_sys_status.onboard_control_sensors_present,
      onboardControlSensorsEnabled:
          _bindings.rx_sys_status.onboard_control_sensors_enabled,
      onboardControlSensorsHealth:
          _bindings.rx_sys_status.onboard_control_sensors_health,
      load: _bindings.rx_sys_status.load,
      voltageBattery: _bindings.rx_sys_status.voltage_battery,
      currentBattery: _bindings.rx_sys_status.current_battery,
      batteryRemaining: _bindings.rx_sys_status.battery_remaining,
      dropRateComm: _bindings.rx_sys_status.drop_rate_comm,
      errorsComm: _bindings.rx_sys_status.errors_comm,
      errorsCount1: _bindings.rx_sys_status.errors_count1,
      errorsCount2: _bindings.rx_sys_status.errors_count2,
      errorsCount3: _bindings.rx_sys_status.errors_count3,
      errorsCount4: _bindings.rx_sys_status.errors_count4,
    );

    mavlinkGpsStatus = MavlinkGpsStatus(
      satellitesVisible: _bindings.rx_gps_status.satellites_visible,
      satelliteUsed:
          convertFfiArrayToListInt(_bindings.rx_gps_status.satellite_used, 20),
      satellitePrn:
          convertFfiArrayToListInt(_bindings.rx_gps_status.satellite_prn, 20),
      satelliteElevation: convertFfiArrayToListInt(
          _bindings.rx_gps_status.satellite_elevation, 20),
      satelliteAzimuth: convertFfiArrayToListInt(
          _bindings.rx_gps_status.satellite_azimuth, 20),
      satelliteSnr:
          convertFfiArrayToListInt(_bindings.rx_gps_status.satellite_snr, 20),
    );

    mavlinkAttitude = MavlinkAttitude(
      timeBootMs: _bindings.rx_attitude.time_boot_ms,
      roll: _bindings.rx_attitude.roll,
      pitch: _bindings.rx_attitude.pitch,
      yaw: _bindings.rx_attitude.yaw,
      rollSpeed: _bindings.rx_attitude.rollspeed,
      pitchSpeed: _bindings.rx_attitude.pitchspeed,
      yawSpeed: _bindings.rx_attitude.yawspeed,
    );

    mavlinkGlobalPositionInt = MavlinkGlobalPositionInt(
      timeBootMs: _bindings.rx_global_position_int.time_boot_ms,
      lat: _bindings.rx_global_position_int.lat,
      lon: _bindings.rx_global_position_int.lon,
      alt: _bindings.rx_global_position_int.alt,
      relativeAlt: _bindings.rx_global_position_int.relative_alt,
      vx: _bindings.rx_global_position_int.vx,
      vy: _bindings.rx_global_position_int.vy,
      vz: _bindings.rx_global_position_int.vz,
      hdg: _bindings.rx_global_position_int.hdg,
    );

    mavlinkLocalPositionNed = MavlinkLocalPositionNed(
      timeBootMs: _bindings.rx_local_position_ned.time_boot_ms,
      x: _bindings.rx_local_position_ned.x,
      y: _bindings.rx_local_position_ned.y,
      z: _bindings.rx_local_position_ned.z,
      vx: _bindings.rx_local_position_ned.vx,
      vy: _bindings.rx_local_position_ned.vy,
      vz: _bindings.rx_local_position_ned.vz,
    );

    mavlinkMissionRequestInt = MavlinkMissionRequestInt(
      targetSystem: _bindings.rx_mission_request_int.target_system,
      targetComponent: _bindings.rx_mission_request_int.target_component,
      seq: _bindings.rx_mission_request_int.seq,
      missionType: _bindings.rx_mission_request_int.mission_type,
    );

    if (res == 40) {
      mavlinkMissionRequest = MavlinkMissionRequest(
        targetSystem: _bindings.rx_mission_request.target_system,
        targetComponent: _bindings.rx_mission_request.target_component,
        seq: _bindings.rx_mission_request.seq,
        missionType: _bindings.rx_mission_request.mission_type,
      );
    }

    mavlinkMissionAck = MavlinkMissionAck(
        targetSystem: _bindings.rx_mission_ack.target_system,
        targetComponent: _bindings.rx_mission_ack.target_component,
        type: _bindings.rx_mission_ack.type,
        missionType: _bindings.rx_mission_ack.mission_type,
        opaqueId: 0);

    mavlinkStatusText = MavlinkStatusText(
      severity: _bindings.rx_statustext.severity,
      chunkSeq: _bindings.rx_statustext.chunk_seq,
      id: _bindings.rx_statustext.id,
      text: convertFfiArrayToString(_bindings.rx_statustext.text, 50),
    );

    messages.add(mavlinkHeartbeat!);
    messages.add(mavlinkSystStatus!);
    messages.add(mavlinkGpsStatus!);
    messages.add(mavlinkAttitude!);
    messages.add(mavlinkGlobalPositionInt!);
    messages.add(mavlinkLocalPositionNed!);
    messages.add(mavlinkMissionRequestInt!);
    if (res == 40) {
      messages.add(mavlinkMissionRequest!);
    }
    messages.add(mavlinkMissionAck!);
  }

  return messages;
}

String convertFfiArrayToString(Array<Char> ffiArray, int length) {
  List<int> resultList = [];

  // Перебор элементов ffi.Array и добавление их в список
  for (int i = 0; i < length; i++) {
    if (ffiArray[i] == 0) break;
    resultList.add(ffiArray[i]);
  }

  return String.fromCharCodes(resultList);
}

List<int> convertFfiArrayToListInt(Array<Uint8> ffiArray, int length) {
  // Инициализация пустого списка для хранения результата
  List<int> resultList = [];

  // Перебор элементов ffi.Array и добавление их в список
  for (int i = 0; i < length; i++) {
    resultList.add(ffiArray[i]);
  }

  return resultList;
}

/// The dynamic library in which the symbols for [RtkMavlinkBindings] can be found.
final DynamicLibrary _dylib = () {
  if (Platform.isMacOS || Platform.isIOS) {
    return DynamicLibrary.open('$_libName.framework/$_libName');
  }
  if (Platform.isAndroid || Platform.isLinux) {
    return DynamicLibrary.open('lib$_libName.so');
  }
  if (Platform.isWindows) {
    return DynamicLibrary.open('$_libName.dll');
  }
  throw UnsupportedError('Unknown platform: ${Platform.operatingSystem}');
}();

/// The bindings to the native functions in [_dylib].
final RtkMavlinkBindings _bindings = RtkMavlinkBindings(_dylib);
