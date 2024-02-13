# rtk_mavlink

Плагин для взаимодействия пульта и устройства на ОС Аврора.

## Установка 

Вариант 1. Для начала плагин нужно добавить в проект. В файл pubspec.yaml добавить следующие строчки:
```yaml
dependencies:
  rtk_mavlink:
      git:
        url: https://github.com/Hatemsla/rkt_mavlink.git
```

Вариант 2. Если плагин ставится напрямую, то есть исходники будут лежать в приложении.

```yaml
dependencies:
  rtk_mavlink:
    path: plugins/rtk_mavlink
```

## Использование

Выполнить импорт плаигна в код:
```dart
import 'package:rtk_mavlink/rtk_mavlink.dart';
```


Далее, можно вызывать методы описанные в файле rtk_mavlink.h

Пример вызова функции:
```dart
String message = point_is_reached().toString();
debugPrint(message);
```
