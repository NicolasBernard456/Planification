#ifndef GRAPH_HPP
#define GRAPH_HPP
#include <ros/ros.h>
#include "Noeud.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
class Graph{
	private : 
		std::vector<Noeud> graph;
		std::vector<int> poid;
		std::vector<bool> visit;
		ros::Publisher pub;
		
		
		
	public:
		Graph();
		~Graph();
		void init_ros();
		void Add_noeud(const Noeud& n);
		const int get_size();
		Noeud getNoeud(int indice);
		bool init_connexion();
		void display();
		void dijkstra(bool display, nav_msgs::OccupancyGrid map, int id_depart, int id_arrivee);	
// 		int min_poid();
		int recherche_noeud_min();
		bool condition_noeud_fils(Noeud* fils, Noeud* pere);
		float dist_Noeuds(Noeud* n1, Noeud* n2);
};



#endif