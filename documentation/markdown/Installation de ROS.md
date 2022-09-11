# Installation de ROS

## Préambule :

Ce Wiki contient les étapes d’installation de ROS et de Gazebo.

Le système d'exploitation utilisé ici est Linux Mint 20.3 (Una), c'est simplement une distribution qui repose sur Ubuntu 20.04 (Focal). Si vous disposez d'un autre système d'exploitation, je mettrai les liens vers les pages d'installation de chaque outil pour que vous puissiez choisir le tutoriel correspondant.

## Installation et configuration de ROS :

### Liens d’installation

Ce tutoriel concerne l’installation de ROS 1 sur lequel le projet a été réalisé (il est recommandé de migrer vers ROS 2 dans le futur, vu que cette version va bientôt être la plus adoptée). La distribution de ROS 1 utilisée pour *Ubuntu focal* est la distribution *Noetic* dont le tutoriel d’installation est disponible ici : 

[Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)

Si vous avez un autre système d’exploitation ou que vous voulez installer une nouvelle version de ROS 1, veuillez suivre un des tutoriels trouvés dans cette page : 

[Wiki](http://wiki.ros.org/ROS/Installation)

### Installation

Retranscrites du premier tutoriel, voici les commandes (en condensé) à exécuter pour que votre système d’exploitation puisse recevoir les paquets à partir de ROS :

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

<aside>
💡 **Note :** Si vous utilisez Linux Mint (ou autre distribution par dessus Ubuntu), remplacez la commande `$(lsb_release -sc)` par le nom de code de la distribution Ubuntu sur lequel votre système d'exploitation est basé (ex : pour Linux Mint **Una**, le nom de code de la distribution d’Ubuntu correspondante est **focal**)

</aside>

La prochaine étape est d’installer ROS, vous avez le choix entre 3 versions : 

- **La version Desktop Full (ROS + Outils (rviz, rqt, etc.) + Gazebo etc.)**
- **La version Desktop (ROS + Outils)**
- **La version Basique (ROS)**

Nous allons installer la version Desktop Full avec cette commande :

```bash
sudo apt install ros-noetic-desktop-full
```

Détendez-vous et laisser ROS s’installer.

### Configuration de l’environnement ROS

Une fois ROS installé, vous devriez pouvoir utiliser les commandes de ROS (comme `roscore` ou `rosrun`) dans n’importe quel terminal. 

Normalement, pour faire en sorte que votre terminal puisse voir l’environnement ROS, vous devez exécuter la commande suivante **à chaque fois que vous lancez un nouveau terminal :**

```bash
source /opt/ros/noetic/setup.bash
```

Les commandes ne sont pas automatiquement visibles dès l’installation pour une raison : comme ça vous pourriez avoir plusieurs versions de ROS en même temps dans un système d’exploitation sans risques de conflits (vous pouvez même avoir deux versions de ROS différentes dans deux terminaux différents). 

Si vous voulez que la commande précédente se lance automatiquement à chaque lancement d’un nouveau terminal, ajoutez simplement la commande en bas du fichier `~/.bashrc`, ou plus simplement lancez ces commandes qui accomplissent la même chose : 

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

<aside>
💡 **Note :** Rappelez-vous de la notion de “sourcing”, car elle est très importante dans l’écosystème ROS. Si vous n’arrivez pas à trouver un package, un noeud ou tout autre élément de ROS dans votre terminal c’est très probablement que vous n’avez pas sourcé un fichier qu’il faut sourcer.

</aside>

### Installation d’outils importants

D’après le lien du tutoriel, il est important d’installer ces outils pour le bon fonctionnement de ROS :

```bash
# Install dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize Rosdep
sudo rosdep init
rosdep update
```

## Gazebo :

### Installation

Si vous avez installé la version Desktop-full de ROS, vous devriez avoir gazebo à sa dernière version, sinon vous pouvez installer gazebo à l’aide de cette commande :

Puis installez ces dépendances : 

### Configuration de l’environnement

Ajoutez cette ligne à votre `~/.bashrc`:

```bash
source /usr/share/gazebo/setup.bash
```