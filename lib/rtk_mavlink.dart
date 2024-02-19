// ignore_for_file: depend_on_referenced_packages

import 'dart:ffi';
import 'dart:io';
import "package:ffi/ffi.dart";

import 'rtk_mavlink_bindings_generated.dart';

const String _libName = 'rtk_mavlink';

String getHeartbeatStringResult() {
  try {
    var heartbeatPointer =
        _bindings.heartbeat_str_result.cast<Utf8>().toDartString();

    return heartbeatPointer;
  } catch (e) {
    return '';
  }
}

String getSysStatusStringResult() {
  try {
    var sysStatusPointer =
        _bindings.sys_status_str_result.cast<Utf8>().toDartString();

    return sysStatusPointer;
  } catch (e) {
    return '';
  }
}

String getGpsStatusStringResult() {
  try {
    var gpsStatusPointer =
        _bindings.gps_status_str_result.cast<Utf8>().toDartString();

    return gpsStatusPointer;
  } catch (e) {
    return '';
  }
}

String getAttitudeStringResult() {
  try {
    var attitudePointer =
        _bindings.attitude_str_result.cast<Utf8>().toDartString();

    return attitudePointer;
  } catch (e) {
    return '';
  }
}

String getGlobalPositionIntStringResult() {
  try {
    var globalPositionPointer =
        _bindings.global_position_int_str_result.cast<Utf8>().toDartString();

    return globalPositionPointer;
  } catch (e) {
    return '';
  }
}

void updateData(List<int> newBytes) {
  for (var i = 0; i < newBytes.length; i++) {
    _bindings.update_data(newBytes[i]);
  }
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
