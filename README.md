# FAURSA Project 
## Téléchargement du projet

Vérifiez que vous disposez de git dans votre système d’exploitation en exécutant la commande : 

```bash
git
```

Si le terminal vous dit que la commande n’a pas été retrouvée, lancez la commande suivante pour l’installer : 

```bash
sudo apt install git
```

Une fois la commande installée, déplacez-vous vers le dossier de votre choix (le dossier où sera placé le projet) et lancez la commande : 

```bash
git clone https://github.com/Moncef-Boukhecheba/FAURSA-Project
```

Cette commande installe le projet sans les sous-modules (submodules), les sous-modules sont optionnels et sont les suivants : 

- **aws-robomaker-hospital-world :** Une collection de mondes et de modèles 3D autour du thème de l’hôpital, créés pour être utilisés dans une simulation sous Gazebo.
- **LaMa (ne fonctionne pas pour l’instant) :** Ce sous-module est un package utilisé pour la création de mapping et / ou localisation (SLAM). Par défaut le projet se lance avec une carte pré-enregistrée d’un monde simple donc aucun mapping n’est nécessaire. D’autres modules sont d’ailleurs utilisés pour SLAM notamment gmapping et hector_mapping donc LaMa n’est qu’un plus.
- **gazebo_ros_2Dmap_plugin (necessite la stack navigation)** : Ce sous-module permet de construire des cartes du monde à partir de gazebo.

Si vous voulez cloner **un seul sous-module,** exécutez cette commande dans la racine du projet : 

```bash
git submodule update --init -- <chemin_vers_le_dossier_du_sous_module>

# Exemple 
git submodule update --init -- simulation_ws/src/iris_lama_ros
```

Sinon, pour télécharger le projet avec tous ses sous-modules, lancez la commande : 

```bash
git clone https://github.com/Moncef-Boukhecheba/FAURSA-Project --recurse-submodules
```

## Liens notion : 
### Wiki du projet FAURSA :
https://ballistic-hydrangea-dee.notion.site/Wiki-projet-FAURSA-bf9e1063d8d2465b87a69141c96baeb7

### Installation et configuration de ROS :
https://ballistic-hydrangea-dee.notion.site/Installation-de-ROS-0105ac31cb9c425e8968b1b4e6449a9d

