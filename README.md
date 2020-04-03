# Platoons

## Resources

| **Resource**       | **Description**   |
|--------------------|:------------------|
| [G-Drive](https://drive.google.com/drive/folders/1yUdHC7Go0_v80fDRuy0mRORAJV-xAzHh?usp=sharing)	| Google Drive for Management and Literature	|
| [Overleaf](https://www.overleaf.com/1737169256jszkkfntpjtb)						| Overleaf Project for Paper Writing		|
| [Github-repo](https://github.com/eigenomarksamy/platoons.git)						| Github Repository for Source Code		|

## Directory Management

The following list shows the directories and explains their purpose:

* **plat_app**	-> *Directory responsible for the packages that will be controlled by the user to perform a demonstration*
	- `plat_cfg`	-> *Package responsible for sending configurations of the demonstration into the ROS Parameter Server*
* **plat_com**	-> *Directory responsible for handling the communication inside and outside the platoon*
	- `plat_frm`	-> *Package responsible for managing the external vehicles messages and forming the platoon*
* **plat_sdc**	-> *Directory responsible for the self-driving behavior of each of the vehicles*
	- `plat_loc`	-> *Package responsible for each of the vehicles localization*
	- `plat_nav`	-> *Package responsible for each of the vehicles navigation*
* **plat_sim**	-> *Directory responsible for handling the simulation environment*
	- `plat_dbw`	-> *Package responsible for interfacing with the drive commands and sensors feedback for each vehicle*
	- `plat_gaz`	-> *Package responsible for interfacing and launching the visualization models*
	- `plat_map`	-> *Package responsible for configuring the map in the simulation*
	- `plat_msg`	-> *Package responsible for custom messages*
