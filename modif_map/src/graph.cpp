#include "graph.hpp"


Graph::Graph(){

}


Graph::~Graph()
{

}

void Graph::init_ros(){
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;
	pub = nh.advertise<nav_msgs::Path>("/Waypoints_robot", 1);
}


void Graph::Add_noeud(const Noeud& n){
	graph.push_back(n);
}

const int Graph::get_size(){
	return graph.size();
}

Noeud Graph::getNoeud(int indice){
	if(indice < graph.size())
		return graph[indice];
	else{
		std::cout << "Erreur indice hors du tableau" << std::endl;
		return Noeud();
	}
}

bool Graph::init_connexion(){	//Init les connexions pour les deux dernieres positions entrees
	bool res = false;
	for(int i = 0 ; i < get_size() ; i++){
		for(int j = 0; j < get_size() ; j++){
			if(graph[i].check_adjac_es(&(graph[j])))
				res = true;;
		}
	}
	return res;
}


void Graph::display(){
	if(get_size() == 0){
		ROS_ERROR("Erreur graph vide");
		return;
	}
	for(int i = 0; i < get_size() ; i++)
		graph[i].create_output();
	graph[get_size() - 1].display();

}



int Graph::recherche_noeud_min(){	//Recherche le poid min dans toutes les cases non visité
	int min_poid = -1, indice = -1;
	for(int i = 0 ; i < get_size() ; i++){
		if(visit[i] == false && (min_poid == -1 || (min_poid > poid[i]) && poid[i] != -1)){
			min_poid = poid[i];
			indice = i;
		}
	}
	visit[indice] = true;
	return indice;
}

bool Graph::condition_noeud_fils( Noeud* fils, Noeud* pere){
	if( (!visit[fils->get_num_noeud()]))
		if( (poid[pere->get_num_noeud()] + 1 < poid[fils->get_num_noeud()]) || poid[fils->get_num_noeud()] == -1)
			return true;
	return false;
	
}


void Graph::dijkstra(bool display, nav_msgs::OccupancyGrid map, int id_depart, int id_arrivee ){

	for(int i = 0 ; i < get_size(); i++){
		poid.push_back(-1);
		visit.push_back(false);
	}
	//On place le poid du Noeud 0 à 0 pour dire qu'on commence ici (premiers tests) Utiliser les arguments de la fonction
	poid[id_depart] = 0;
	Noeud* pere;
	do{
		pere = &(graph[recherche_noeud_min()]);
		std::vector<Noeud*> fils = pere->get_adjac();
		for(int i = 0 ; i < fils.size() ; i++){
			if(condition_noeud_fils(fils[i],pere)){
				poid[fils[i]->get_num_noeud()] = poid[pere->get_num_noeud()] + 1;
				fils[i]->set_antecedent(pere);
			}
		}
	}while(pere->get_num_noeud() != id_arrivee);
	nav_msgs::Path path;
	geometry_msgs::PoseStamped pose;
	path.header.frame_id = "/map";
	std::vector <geometry_msgs::PoseStamped> pose_inverse;
	Noeud* antecedent = graph[id_arrivee].get_antecedent();
	if(display)
		graph[id_arrivee].color_chemin();
	pose.pose.position.x = graph[id_arrivee].getX() * map.info.resolution + map.info.origin.position.x;
	pose.pose.position.y = graph[id_arrivee].getY() * map.info.resolution + map.info.origin.position.y;
	pose_inverse.push_back(pose);
	while( antecedent->antecedent_exist()){
		if(display)
			antecedent->color_chemin();
		pose.pose.position.x = antecedent->getX() * map.info.resolution + map.info.origin.position.x;
		pose.pose.position.y = antecedent->getY() * map.info.resolution + map.info.origin.position.y;
		pose_inverse.push_back(pose);
		antecedent = antecedent->get_antecedent();
	}
	for(int i = 0 ; i < pose_inverse.size() ; i++){
		path.poses.push_back(pose_inverse[pose_inverse.size() - i - 1]);
	}
	pub.publish(path);
	
}



