Репозиторий содержит ROS1 интерфейс для библиотеки семантической сегментации semseg ([gitlab](https://gitlab.com/sdbcs-nio3/itl_mipt/segm_tracking/alg/2d_segmentation/semseg), [github](https://github.com/cds-mipt/semseg), хотя бы к одной версии должен быть доступ) и является частью примера интеграции нейросетевого модуля в ROS1.

## Создание рабочего пространства

Официальные туториалы по созданию рабочего пространства в ROS1: 
1. http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
2. http://wiki.ros.org/catkin/workspaces

Исходный код и артефакты сборки принято хранить в рабочем пространстве. В ROS1 принято его называть catkin_ws, а исходный код (в виде набора пакетов/репозиториев) хранить в catkin_ws/src
```bash
mkdir ~/workspace/catkin_ws/src -p
```

Далее можно склонировать исходный код необходимых пакетов:
```bash
cd ~/workspace/catkin_ws/src

# с gitlab
git clone https://gitlab.com/sdbcs-nio3/itl_mipt/segm_tracking/alg/2d_segmentation/semseg.git
git clone https://gitlab.com/sdbcs-nio3/itl_mipt/segm_tracking/alg/2d_segmentation/semseg_ros1.git

# или, если нет доступа к gitlab, с github
git clone git@github.com:cds-mipt/semseg.git
git clone git@github.com:cds-mipt/semseg_ros1.git
```


## docker

Офизиальный туториал по docker: https://docs.docker.com/get-started/

Для упрощения развертывания узлов используются docker образы. Такой образ создается по Dockerfile, поэтому сначала необходимо подготовить его. За основу можно взять представленный в данном репозитории в директории docker.

Развертывание узлов может производиться на разных платформах. При этом Dockerfile может отличаться для разных платформ. Для сборки на обычном компьютере название файла должно быть `Dockerfile.x86_64`, для сборки на Jetson - `Dockerfile.aarch64`.

Первая строка Dockerfile:
```dockerfile
FROM nvcr.io/nvidia/cuda:11.1.1-devel-ubuntu20.04
```
обозначает базовый образ. На основе него будет производиться сборка образа для развертывания узлов. Для работы с нейросетевыми моделями нужны такие библиотеки как CUDA, CuDNN и др. А также в контейнере должен быть установлен ROS2. Удобно выбрать базовый образ, в котором установлены необходимые библиотеки для работы с моделями, а ROS1 доустановить, указав инструкции по его установке в Dockerfile. Список образов с предустановленными библиотеками можно найти на https://ngc.nvidia.com/catalog/containers. Для Jetson необходимо использовать образы с L4T (Linux for Tegra) в названии. ROS1 Noetic может быть установлен добавлением в Dockerfile следующих строк:
```dockerfile
# Install ROS1 Noetic
RUN apt-get update && apt-get install --no-install-recommends -y \
        gnupg \
        lsb-release \
        curl && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install --no-install-recommends -y \
        ros-noetic-ros-base \
        ros-noetic-cv-bridge \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*
```

Вторая строка Dockerfile:
```dockerfile
ENV DEBIAN_FRONTEND noninteractive
```
отключает интерактивный режим, который недопустим на этапе сборки. Отсутствие этой строки может привести к зависанию сборки, так как будет ожидаться ввод данных пользователем, что невозможно на этом этапе.

Другим важным элементом является создание нового пользователя внутри контейнера, от имени которого будут производиться все действия:
```dockerfile
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PW}" | chpasswd && adduser ${USER} sudo
WORKDIR /home/${USER}
RUN mkdir -p catkin_ws/src && chown -R ${UID}:${GID} /home/${USER}
USER ${UID}:${GID}
```
Как правило, эти строки можно использовать без изменений и в других репозиториях, изменив лишь имя пользователя в строке:
```dockerfile
ARG USER=docker_semseg
```
на более подходящее.

Также стоит обратить внимание на строчку
```dockerfile
ENV PYTHONPATH=/home/${USER}/catkin_ws/src/semseg:${PYTHONPATH}
```
Она позволяет осуществлять import python-пакета semseg, реализующего библиотеку семантической сегментации. Это наиболее простой путь решить проблему вида: `ImportError: no module named semseg`. Друим возможным решением является добавление инструкций по установке и сама установка пакета semseg. Если используемая библиотека их уже содержит, то можно воспользоваться ими. Иначе придется добавлять самостоятельно, что может существенно зависеть от конкретной библиотеке.

Все остальные строки Dockerfile - это инструкции по установке зависимостей библиотеки semseg и ROS1 обвязки semseg_ros1.

Кроме Dockerfile необходимо также создать набор скриптов для сборки (build.sh), запуска (start.sh) и входа (into.sh) в контейнер. Как правило, эти скрипты можно использовать и в других репозиториях, изменив лишь имя пользователя на указанное в Dockerfile, а имя образа и имя контейнера на более подходящие.

В итоге, последовательность работы с docker обвязкой должна получиться следующей:
```bash
bash build.sh
bash start.sh
bash into.sh
```
В результате выполнения этой последовательности команд должен быть создан образ, на его основе запущен контейнер и осуществлен в него вход.

Для дальнейшей работы с пакетом необходимо их выполнить в консоли.


## Работа с готовым пакетом

В настоящем репозитории представлен готовый к использованию ROS1 пакет семантической сегментации. 

К данному моменту предполагается, что собран образ, запущен контейнер и выполнен вход в него. После этого последовательность команд, необходимая для запуска, должна быть максимально простой. 

Сначала необходимо собрать пакет:
```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release --only-pkg-with-deps semseg_ros1
source devel/setup.bash
```
Затем запустить launch, который автоматически запустит необходимые компоненты:
```bash
roslaunch semseg_ros1 semseg.launch
```

Нужно стремиться к тому, чтобы последовательность работы с готовым пакетом состояла только из этих четырех строк. Все остальное должно быть перенесено на этап сборки образа, то есть в Dockerfile.

launch файл сконфигурирован для запуска на датасете KITTI. Чтобы запустить проигравание BAG файла, нужно сначала [скачать](https://drive.google.com/file/d/1pfzTmBGHje55STJNKxfkVbQE8ylg-6ds/view?usp=sharing) его.
Для запуска проигрывания нужно сначала активировать окружение ROS1:
```bash
source /opt/ros/noetic/setup.bash
ros2 bag play --clock kitti_2011_10_03_drive_0027_synced.bag
```
Визуализировать результаты работы можно либо с помощью rqt
```bash
source /opt/ros/noetic/setup.bash
rqt
# Plugins -> Visualization -> Image View, в выпадающем списке выбрать желаемый топик
```
либо с помощью rviz:
```bash
source /opt/ros/noetic/setup.bash
rviz
# Add -> By topic, в списке выбрать желаемый топик (Image)
```

## Создание пакета с нуля

Официальные туториалы по созданию пакета: 
1. http://wiki.ros.org/ROS/Tutorials/CreatingPackage
2. http://wiki.ros.org/ROS/Tutorials/BuildingPackages
3. http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

Официальный туториал по работе с launch: 
1. http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch

Далее рассмотрим последовательность действий, которую необходимо выполнить, чтобы создать ROS1 пакет семантической сегментации с нуля, имея код библиотеки семантической сегментации semseg. Но так как код пакета уже представлен в репозитории, его придется сначала удалить из него:
```bash
rm -rf semseg_ros1
```

К данному моменту предполагается, что собран образ, запущен контейнер и выполнен вход в него.

Сначала необходимо создать шаблон пакета, с помощью команды:
```bash
cd ~/catkin_ws/src/semseg_ros1
catkin_create_pkg --rosdistro noetic semseg_ros1 rospy
```
Созданные при этом файлы и их содержимое содержатся в коммите [c6df9a70](https://gitlab.com/sdbcs-nio3/itl_mipt/segm_tracking/alg/2d_segmentation/semseg_ros1/-/commit/c6df9a7086c82e4c8920a7b99d31379028d45744).

Далее нужно создать необходимые узлы, добавив файлы с исходным кодом, как это было сделано в коммитах [240858da](https://gitlab.com/sdbcs-nio3/itl_mipt/segm_tracking/alg/2d_segmentation/semseg_ros1/-/commit/240858dacf87a799d8ada6076ab9a1b8d1483120), [b2bb5922](https://gitlab.com/sdbcs-nio3/itl_mipt/segm_tracking/alg/2d_segmentation/semseg_ros1/-/commit/b2bb5922c561e8e86f1a0eadfb02cadf03384de1). Также созданные файлы нужно сделать исполняемыми:
```bash
chmod +x <путь к файлу>
```
Иначе будет ошибка вида Cannot locate node of type [...] in package [...]

Последним этапом является добавление launch файлов (но можно их добавить и раньше), как это сделано в коммите [e8300c93](https://gitlab.com/sdbcs-nio3/itl_mipt/segm_tracking/alg/2d_segmentation/semseg_ros1/-/commit/e8300c935627cf4d5aac05bbff800f973b77f1ff)

После этого можно перейти к пункту Работа с готовым пакетом для проверки работоспособности.
