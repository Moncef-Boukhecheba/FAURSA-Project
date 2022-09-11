# Wiki projet FAURSA

## Lancement du projet

Après le téléchargement du projet, lancez le fichier `first_setup.sh` qui se trouve à la racine du projet, ce fichier installe **toutes** les dépendances (mêmes optionnelles), construit tous les packages à l’aide de catkin, et source les fichiers nécessaires :

```bash
source first_setup.sh
```

💡 **Note :** Si vous ne souhaitez pas installer les dépendances (à savoir `ros-noetic-map-server ros-noetic-amcl ros-noetic-gmapping ros-noetic-hector-mapping`) vous pouvez lancer le fichier `make_projects.sh` et choisir d’installer les dépendances une par une.


A chaque fois que vous ouvrez un nouveau terminal, lancez la commande suivante pour que les packages et les noeuds créés dans le projet soient visibles par ROS :

```bash
source setup.sh
```

## Lancement de la simulation + Rviz

Pour lancer la simulation et Rviz avec un des mondes utilisés dans le projet, utilisez le paramètre **world_name**, par exemple avec la commande : 

```bash
roslaunch wheelchair wheelchair.launch world_name:=[NOM_DU_MONDE]
```

Les mondes utilisés dans ce projet sont les suivants : 

- **HEXAGON WORLD (monde par défaut, tapez hexagon) :**

![world_hexagon.png](Wiki%20projet%20FAURSA/world_hexagon.png)

- **SIMPLE HOUSE (tapez simple_house) :**

![world_house.png](Wiki%20projet%20FAURSA/world_house.png)

- **AWS ROBOMAKER HOSPITAL WORLD (tapez hospital) :**

![hospital_world.png](Wiki%20projet%20FAURSA/hospital_world.png)

La chaise est instanciée dans des positions différentes selon le monde, définies dans le fichier **launch/wheelchair.launch,** mais il est possible de définir une position initiale différente avec le paramètre **pos_x** et **pos_y** :

```bash
roslaunch wheelchair wheelchair.launch pos_x:=[POSITION_X] pos_y:=[POSITION_Y]
```

Quatre **modes de mapping** sont implémentés dans ce projet dont trois algorithmes SLAM, pour spécifier lequel de ces modes utiliser, utilisez le paramètre **mapping_mode**, par exemple en tapant cette commande : 

```bash
roslaunch wheelchair wheelchair.launch mapping_mode:=[MODE_DE_MAPPING]
```

Les modes de mapping sont les suivants : 

- **GMapping (tapez “gmapping”)**
- **Hector Mapping (tapez “hector”)**
- **Large Maps Framework, LaMa (tapez “lama”)**
- **Utiliser la carte du monde créée au préalable avec gmapping (par défaut, tapez “saved”)**
- **Aucun mapping (tapez “none”)**

Si vous utilisez un des algorithmes SLAM et une fois la carte générée si vous voulez **sauvegarder la carte,** mettez-vous dans le dossier **simulation_ws/src/wheelchair/map** et ****utilisez cette commande :

```bash
rosrun map_server map_saver -f [NOM_DU_MONDE]
```

💡 **Note :** Il est **impératif** d’utiliser le même nom pour le monde et le fichier de la carte enregistré, sinon vous aurez une erreur en lançant le fichier **wheelchair.launch**.

## Lancement et utilisation des techniques de navigation :

Une fois votre environnement de simulation créé, pour tester les méthodes de navigation implémentées, tapez cette commande :

```bash
rosrun motion_planning [NOM_DU_NOEUD]
```

Deux noeuds principaux sont disponibles : 

- **iss :** utilise l’algorithme ISS (Input Space Sampling) pour la navigation
- **optimisation_local :** utilise un algorithme d’optimisation pour la navigation, pour spécifier l’algorithme de navigation, tapez la commande :

```bash
rosrun motion_planning optimisation_local _algorithm:=[NOM_ALGORITHME]
```

Les algorithmes disponibles sont : 

- **Particule Swarm Optimization (PSO, tapez PSO)**
- **Whale Optimization Algorithm (WOA, tapez WOA)**
- (BIENTOT) **Grey Wolf Optimizer (GWO, tapez GWO)**
