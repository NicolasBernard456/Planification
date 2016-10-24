#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "Noeud.hpp"
#include "graph.hpp"
#include <boost/graph/adjacency_list.hpp>
ros::Publisher pub;

cv::Mat image;

bool display;
int param_div;


geometry_msgs::PointStamped depart;
geometry_msgs::PointStamped arrive;
int cpt_es;	//Compteur entrée sortie.
nav_msgs::OccupancyGrid new_map_temp;
int cpt_noeud;
Graph g;


bool obstacle(std::vector<int> map){
	for(int i = 0 ; i < map.size(); i++){
		if(map[i] != 0)
			return false ;
	}
	return true;
}

nav_msgs::OccupancyGrid init_map_image(nav_msgs::OccupancyGrid map, int* coord_extremum){
	nav_msgs::OccupancyGrid new_map;
	new_map.header = map.header;
	new_map.info.origin = map.info.origin;
	new_map.header.frame_id = map.header.frame_id;
	new_map.info.width = map.info.width;
	new_map.info.height = map.info.height;
	new_map.info.resolution = map.info.resolution;
	new_map.data.assign(new_map.info.height * new_map.info.width,-1);
	FILE* out;
	if(display){
		std::string mapdatafile = "test.pgm";
		out = fopen(mapdatafile.c_str(), "w");
		if (!out || map.data.size() == 0){
			std::cout << "Couldn't save map file to %s" << std::endl;
			return new_map;
		}
	
	fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n", map.info.resolution, map.info.width, map.info.height);
	}
	for(unsigned int y = 0; y < map.info.height; y++) {
		for(unsigned int x = 0; x < map.info.width; x++) {
			unsigned int i = x + (map.info.height - y - 1) * map.info.width;	//Le repere part d'en bas a gauche sur la carte alors que l'image part d'en haut a gauche
			if (map.data[i] < 0)
				new_map.data[i] = 100;
			else if (map.data[i] == 100)
				new_map.data[i] = 100;
			else{
				//Optimisation, on regarde quand commence la carte pour éviter de passer sur des zones inutiles
				if(coord_extremum[0] > x || coord_extremum[0] == -1)
					coord_extremum[0] = x;
				else if(coord_extremum[2] < x || coord_extremum[2] == -1)
					coord_extremum[2] = x;
				if(coord_extremum[1] > map.info.height - y - 1 || coord_extremum[1] == -1)
					coord_extremum[1] = map.info.height - y - 1;
				else if(coord_extremum[3] < map.info.height - y - 1 || coord_extremum[3] == -1)
					coord_extremum[3] = map.info.height - y - 1;
				new_map.data[i] = 0;
			}
			if(display){
				if (map.data[i] == 0) { //occ [0,0.1)
					fputc(254, out);
				} else if (map.data[i] == +100) { //occ (0.65,1]
					fputc(000, out);
				} else { //occ [0.1,0.65]
					fputc(205, out);
				}
			}
		}
	}
	if(display){
		fclose(out);
		std::cout << "map saved" << std::endl;
		image = cv::imread("test.pgm", CV_LOAD_IMAGE_COLOR);   // Read the file
		if(! image.data ){
			std::cout <<  "Could not open or find the image" << std::endl ;
			return new_map;
		}
	}
	return new_map;
}

void callback(nav_msgs::OccupancyGrid map_){
	std::cout << "Map_received" << std::endl;
	g.init_ros();
	nav_msgs::OccupancyGrid new_map;
	int i = 0, x2 = 0,y2 = 0;
	std::vector<int> temp;
	int extremum[4] = {-1,-1,-1,-1};
	
	bool obstacle_map;
	new_map = init_map_image(map_, extremum);
	
	for(i = 0 ; i < 4 ; i++)
		if(extremum[i] == -1){
			ROS_ERROR("Erreur lors de la création de la carte, extremum non initialisé");
			return ; //Si les extremum ne sont pas initialisé, il y a eu un probleme. On arrete le prgm
		}
	i= 0;
	for ( int y = extremum[1] ; y < extremum[3] ; y = y + param_div){
		for (int x = extremum[0] ; x < extremum[2] ; x = x + param_div){
			for (x2 = x ; x2 < x + param_div ; x2 ++)
				for (y2 = y ; y2 < y + param_div ; y2 ++)
					temp.push_back(new_map.data[(new_map.info.width)*y2 + x2]);
			if(temp.size() == (param_div * param_div))
				obstacle_map = obstacle(temp);
			else
				obstacle_map = false;
			if(obstacle_map){
				Noeud n;
				n.init_noeud(cpt_noeud++,temp,param_div);
				n.ini_centre(x2,y2);
				if(display)
					n.set_image(image);
				g.Add_noeud(n);
			}
			temp.clear();
		}
	}
	std::cout << "Attente creation connexion entre noeuds" << std::endl;
	g.init_connexion();

	std::cout << "Fin creation connexions" << std::endl;
	new_map_temp = new_map;
	std::cout << "Map sent Graph size is:" << g.get_size() <<  std::endl;
	pub.publish(new_map);
}

void callback2(geometry_msgs::PointStamped ps){
	if(cpt_es == 0){
		depart = ps;
		ROS_INFO("Position de depart recue");
	}
	else if(cpt_es == 1){
		ROS_INFO("Position d'arrivee recue");
		arrive = ps;
		ROS_INFO("Ajout des nouveaux noeuds");
		Noeud n;
		int id_noeud_dep = cpt_noeud++;
		n.init_noeud_es(id_noeud_dep ,(depart.point.x - new_map_temp.info.origin.position.x)/new_map_temp.info.resolution,(depart.point.y - new_map_temp.info.origin.position.y)/new_map_temp.info.resolution,param_div);
		if(display)
			n.set_image(image);
		g.Add_noeud(n);
		Noeud n2;
		int id_noeud_arr = cpt_noeud++;
		n2.init_noeud_es(id_noeud_arr,(arrive.point.x - new_map_temp.info.origin.position.x)/new_map_temp.info.resolution,(arrive.point.y - new_map_temp.info.origin.position.y)/new_map_temp.info.resolution,param_div);
		if(display)
			n2.set_image(image);
		g.Add_noeud(n2);
		ROS_INFO("display image");
		if(!g.init_connexion()){
			ROS_ERROR("Erreur lors de la creation des noeuds, certains noeuds sont isoles");
			ROS_WARN("Reexecuter l'algo avec un pas plus faible");
			return ;
		}

		if(display)
			for(int i = 0; i < g.get_size() ; i++)
				g.getNoeud(i).create_output();
		g.dijkstra(display,new_map_temp,id_noeud_dep, id_noeud_arr);
		if(display)
			g.getNoeud(g.get_size() - 1).display();
		
	}
	cpt_es++;
	if(cpt_es > 1)
		cpt_es = 0;
	
}



int main(int argc, char ** argv){
	ros::init(argc, argv, "test_map");
	ros::NodeHandle n,nh("~");
	nh.param<bool>("display",display,false);
	nh.param<int>("param_div",param_div,100);
	if(display)
		std::cout << "Display on" << std::endl;
	else
		std::cout << "Display off" << std::endl;
	std::cout << "Segmenting map in square of " << param_div << " pixels." << std::endl;
	cpt_es = 0;
	cpt_noeud = 0;
	ros::Subscriber sub = n.subscribe("/map",1 ,callback);
	ros::Subscriber sub2 = n.subscribe("/clicked_point",1 ,callback2);
	pub = n.advertise<nav_msgs::OccupancyGrid>("/new_map", 1);
	ros::spin();
	return 0;
}