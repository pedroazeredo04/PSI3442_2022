# P1 PSI3442

Esse repositório é o pacote ROS que deve ser utilizado para as questões práticas da primeira prova de PSI3442 - Projeto de Sistemas Embarcados de 2021.

## Configuração inicial

O repositório funciona como um pacote ROS, logo deve ser baixado na pasta `src` do seu catkin workspace, de formaa que fique, por exemplo, em `~/catkin_ws/src/p1_psi3442`.

```bash
cd ~/catkin_ws/src/p1_psi3442 # mudar conforme caminho do seu workspace
git clone https://github.com/SkyRats/p1_psi3442.git
```

Também é bom reinstalar os plugins do Gazebo utilizados nessa simulação:

```bash
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
```

```bash
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
```

Além disso, deve-se baixar as dependências de px4. Para uma instalação no ROS Noetic, no Ubuntu 20, recomendo que siga-se as instruções presentes [nesse vídeo](https://www.youtube.com/watch?v=9Mb-aV3lmZ0) (ou, de maneira equivalente, [nesse arquivo de texto](https://kuat-telegenov.notion.site/How-to-setup-PX-toolchain-development-environment-for-drone-simulations-04adcf4370bf4455b374321f5d1e3bb1))

## Utilizando a simulação

Para abrir a simulação, você deve executar os seguintes códigos **dentro da pasta do pacote**.

```bash
cd ~/catkin_ws/src/p1_psi3442

# Esse comando deve ser executado em cada terminal novo
source scripts/setup.bash

# Para abrir a simulação
roslaunch p1_psi3442 simulate.launch

# Para testar: use um comando de decolagem
# No próprio terminal em que executou o comando acima
commander takeoff
```

## Executando código

**Atenção**: depois de modificar os códigos em C++, é necessário recompilar

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

Para executar um programa, basta ter a simulação aberta e utilizar o comando `rosrun`.
Para escolher o caminho a ser tomado, pode-se usar a flag `_flight_route`.

- `_flight_route:=1` - Trajeto que faz as três bordas do quadrado.
- `_flight_route:=2` - Trajeto que faz a diagonal do quadrado.

Assim, o comando para executar o código da P1 fica:

```bash
rosrun p1_psi3442 q1_node _flight_route:=1
```

E o comando para executar o código da P2, fica:

```bash
rosrun p1_psi3442 p2_node
```
