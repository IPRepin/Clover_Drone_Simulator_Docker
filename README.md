# Clover Drone Simulator в Docker

## Описание
Docker-образ для симуляции дрона Clover с использованием ROS Noetic и Gazebo. Образ содержит полностью настроенное окружение для разработки и тестирования программ управления БПЛА.

## Предварительные требования

* Docker
* X Server (для Linux)
* Достаточно мощный компьютер для запуска симуляции (рекомендуется 4+ ядра CPU)

## Установка

1. Установите Docker, если он еще не установлен
2. Скачайте образ:
```bash
docker pull darkus191/clover-simulator:latest
```

## Запуск симулятора

### 1. Разрешите доступ к X Server (только для Linux):
```bash
xhost +local:docker
```

### 2. Базовый запуск с графическим интерфейсом:
```bash
docker run -it --rm \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="GAZEBO_GUI=true" \
    --env="PX4_SIM_SPEED_FACTOR=1.0" \
    -p 8081:80 \
    -p 8082:8080 \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    darkus191/clover-simulator:latest \
    bash -c "source /opt/ros/noetic/setup.bash && \
             source /root/catkin_ws/devel/setup.bash && \
             roslaunch clover_simulation simulator.launch gui:=true maintain_camera_rate:=true"
```

## Доступ к интерфейсам

### Веб-интерфейс
После запуска контейнера доступен по адресам:
* `http://localhost:8081` - основной интерфейс
* `http://localhost:8082` - альтернативный порт

### Gazebo
Графический интерфейс Gazebo запускается автоматически при старте контейнера.

## Настройка параметров запуска

### Производительность
* Изменение скорости симуляции:
  ```bash
  --env="PX4_SIM_SPEED_FACTOR=0.5"  # Замедление симуляции в 2 раза
  ```

### Параметры симуляции
* `gui:=false` - запуск без GUI Gazebo
* `main_camera:=false` - отключение камеры
* `gps:=true` - включение GPS
* `maintain_camera_rate:=true` - стабилизация частоты кадров

### Пример запуска без GUI и камеры
```bash
docker run -it --rm \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="PX4_SIM_SPEED_FACTOR=1.0" \
    -p 8081:80 \
    -p 8082:8080 \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    darkus191/clover-simulator:latest \
    bash -c "source /opt/ros/noetic/setup.bash && \
             source /root/catkin_ws/devel/setup.bash && \
             roslaunch clover_simulation simulator.launch gui:=false main_camera:=false"
```

## Решение проблем

### Низкая производительность
1. Уменьшите `PX4_SIM_SPEED_FACTOR`
2. Отключите камеру (`main_camera:=false`)
3. Отключите GUI (`gui:=false`)
4. Включите `maintain_camera_rate:=true`

### Ошибка доступа к X Server
```bash
xhost +local:docker
```

### Занятые порты
Измените порты в параметрах `-p`, например:
```bash
-p 9090:80 -p 9091:8080
```

## Дополнительные возможности

### QGroundControl
* Настройка параметров дрона
* Планирование миссий
* Управление дроном

### Веб-интерфейс
* Просмотр видео с камеры
* Отправка команд
* Веб-терминал
* Мониторинг состояния

## Примечания по безопасности
* Контейнер запускается с привилегированным доступом
* Используйте только в доверенном окружении
* Соблюдайте осторожность при открытии портов во внешнюю сеть

## Поддержка
При возникновении проблем создавайте Issue в репозитории проекта или обращайтесь к разработчикам.

## Лицензия
[MIT License](LICENSE)
