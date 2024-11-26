# Используем базовый образ Ubuntu 20.04
FROM ubuntu:20.04

# Устанавливаем переменную для неинтерактивного режима
ENV DEBIAN_FRONTEND=noninteractive

# Обновление системы и установка базовых инструментов
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    python3-pip \
    git \
    tzdata \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Установка ROS Noetic
RUN curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && apt-get install -y ros-noetic-desktop-full python3-rosdep && \
    echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

# Инициализируем rosdep
RUN rosdep init && rosdep update

# Устанавливаем переменную ROS_DISTRO глобально
ENV ROS_DISTRO=noetic

# Создаем рабочее пространство catkin
RUN mkdir -p /root/catkin_ws/src && cd /root/catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make" && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

# Клонируем репозитории и устанавливаем зависимости
RUN cd /root/catkin_ws/src && \
    git clone --depth 1 https://github.com/CopterExpress/clover && \
    git clone --depth 1 https://github.com/CopterExpress/ros_led && \
    git clone --depth 1 https://github.com/ethz-asl/mav_comm && \
    cd /root/catkin_ws && \
    rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

# Устанавливаем Python-зависимости
RUN pip3 install -r /root/catkin_ws/src/clover/clover/requirements.txt

# Скачиваем исходный код PX4 и создаем симлинки
RUN git clone --recursive --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git /root/PX4-Autopilot && \
    ln -s /root/PX4-Autopilot /root/catkin_ws/src/ && \
    ln -s /root/PX4-Autopilot/Tools/sitl_gazebo /root/catkin_ws/src/ && \
    ln -s /root/PX4-Autopilot/mavlink /root/catkin_ws/src/

# Устанавливаем зависимости PX4
RUN cd /root/catkin_ws/src/PX4-Autopilot/Tools/setup && ./ubuntu.sh --no-nuttx

# Устанавливаем дополнительные Python-библиотеки
RUN pip3 install toml

# Добавляем раму Клевера
RUN ln -s /root/catkin_ws/src/clover/clover_simulation/airframes/* /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

# Устанавливаем датасеты geographiclib
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# Сборка рабочего пространства
RUN cd /root/catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -j1"

# Установка веб-сервера Monkey
RUN wget https://github.com/CopterExpress/clover_vm/raw/master/assets/packages/monkey_1.6.9-1_amd64.deb -P /tmp && \
    dpkg -i /tmp/monkey_*.deb && rm /tmp/monkey_*.deb && \
    sed "s/pi/$USER/g" /root/catkin_ws/src/clover/builder/assets/monkey | tee /etc/monkey/sites/default && \
    sed -i 's/SymLink Off/SymLink On/' /etc/monkey/monkey.conf

# Команда для запуска симулятора
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && /usr/sbin/monkey -D & roslaunch clover_simulation simulator.launch"]
