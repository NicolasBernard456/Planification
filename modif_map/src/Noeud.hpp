#ifndef NOEUD_HPP
#define NOEUD_HPP

#include <stdlib.h>
#include <vector>
#include <iostream>
#include <cstdio>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


class Noeud {
	private : 
		std::vector<int> data;
		int numero_noeud;
		float cout;
		int x_centre,y_centre;
		std::vector<Noeud*> adjac; // Noeud adjacent
		std::vector<Noeud*> antecedent;
		int size_pix;
		cv::Mat image;
	public :
		Noeud();
		~Noeud();
		void init_noeud(int numero_noeud_, std::vector<int> data_, int size_pix_);
		void init_noeud_es(int numero_noeud_, int x_centre_, int y_centre_, int size_pix_ );
		float get_Cout();
		void ini_centre(int x_centre_, int y_centre_);
		void create_output();
		int get_num_noeud();
		void set_image(cv::Mat image_);
		cv::Mat get_image();
		void display();
		int getX();
		int getY();
		bool check_adjac(Noeud* Noeud_adj);
		std::vector<Noeud*> get_adjac();
		Noeud* get_antecedent();
		void set_antecedent(Noeud* antecedent_);
		void color_chemin();
		bool antecedent_exist();
		bool check_adjac_es(Noeud* Noeud_adj);
		
		
};




#endif