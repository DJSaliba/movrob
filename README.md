# Planejamento de Movimento de Robôs - EEE935 - 2022/2
## Repositório de Trabalhos Práticos

### David Felipe Brochero Giraldo - 2018020395
### Davi Jorge Vanni Saliba - 2018010990

## Configuração do Ambiente

1. A versão do ROS recomendada e utilizada neste repositório é [*Noetic*](http://wiki.ros.org/noetic), em um [*docker Ubuntu 20.04*](https://github.com/D-Brox/ros-docker/tree/linux)
2. Certifique-se de ter configurado um [_workspace_](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) do ROS;
3. Este repositório é um pacote ROS, podendo ser clonado pasta src/ do seu workspace:

```bash
$ cd <workspace>/src
$ git clone https://github.com/D-Brox/movrob
```

Ou extraia o archive

```bash
$ cd <workspace>/src
$ unzip movrob.zip
```


4. Compile seu workspace

```bash
$ cd <workspace>
$ catkin_make
```

5. Execute o setup de desenvolvimento do workspace

```bash
$ source <workspace>/devel/setup.bash
```

## TP1

Obs: caso o stageros não rode, altere os tempos de simulação para 100 nos arquivos `.world`

### Tarefa 1: Tangent Bug

```bash
$ roslaunch movrob TP1_Ex1.launch
```

### Tarefa 2: Trajectory Follower

```bash
$ roslaunch movrob TP1_Ex2.launch
```

### Tarefa 3: Potential Field

```bash
$ roslaunch movrob TP1_Ex3.launch
```

### Tarefa 4: Wave Front

Altere o caminho do arquivo da imagem em `TP1-Ex4.py` e execute:

```bash
$ roslaunch movrob TP1_Ex4.launch
```

