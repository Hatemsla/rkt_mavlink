// ignore_for_file: depend_on_referenced_packages

import 'dart:ffi';
import 'dart:io';
import "package:ffi/ffi.dart";
import 'package:flutter/material.dart';

import 'rtk_mavlink_bindings_generated.dart';

const String _libName = 'rtk_mavlink';

bool isHeartbeatAlreadyRecived = false;

(Array<Uint8>, int) requestAttitude() {
  try {
    var attitudePointer = _bindings.request_attitude();

    debugPrint(attitudePointer.tx_msg_len.toString());

    return (attitudePointer.tx_msg_buffer, attitudePointer.tx_msg_len);
  } catch (e, st) {
    debugPrint(e.toString());
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestSysStatus() {
  try {
    var sysStatusPointer = _bindings.request_sys_status();

    return (sysStatusPointer.tx_msg_buffer, sysStatusPointer.tx_msg_len);
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

(Array<Uint8>, int) requestGpsStatus() {
  try {
    var gpsStatusPointer = _bindings.request_gps_status();

    return (gpsStatusPointer.tx_msg_buffer, gpsStatusPointer.tx_msg_len);
  } catch (e, st) {
    debugPrintStack(stackTrace: st);
    return (const Array<Uint8>(0), 0);
  }
}

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

List<String> updateData(List<int> newBytes) {
  List<String> messages = [];

  for (var i = 0; i < newBytes.length; i++) {
    var message =
        _bindings.update_data(newBytes[i]).cast<Utf8>().toDartString();

    if (message.isNotEmpty) {
      if (message.contains("autopilot") && !isHeartbeatAlreadyRecived) {
        isHeartbeatAlreadyRecived = true;
        messages.add(message);
      } else if (!message.contains('autopilot')) {
        messages.add(message);
      } else {
        continue;
      }
    }
  }

  // _bindings.free_data();

  return messages;
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
