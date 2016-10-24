#include "Noeud.hpp"

#include <stdio.h>

Noeud::Noeud()
{

}

Noeud::~Noeud()
{

}


void Noeud::init_noeud(int numero_noeud_, std::vector< int > data_, int size_pix_){
	numero_noeud = numero_noeud_;
	for(int i = 0; i < data_.size() ; i ++)
		data.push_back(data_[i]);
	size_pix = size_pix_;
}

void Noeud::init_noeud_es(int numero_noeud_, int x_centre_, int y_centre_, int size_pix_){
	numero_noeud = numero_noeud_;
	x_centre = x_centre_;
	y_centre = y_centre_;
	size_pix = size_pix_;
}



float Noeud::get_Cout(){
	return cout;
}

void Noeud::ini_centre(int x_centre_, int y_centre_){
	x_centre = x_centre_ - size_pix/2;
	y_centre = y_centre_ - size_pix/2;
}

int Noeud::get_num_noeud(){
	return numero_noeud;
}

cv::Mat Noeud::get_image(){
	return image;
}

void Noeud::set_image(cv::Mat image_){
	image = image_;
}


void Noeud::create_output(){

	if(! image.data ){
		std::cout <<  "Could not open or find the image" << std::endl ;
		return ;
	}

	cv::Point Pt1, Pt2;
	Pt1.x = x_centre;	//Centre du cercle (x)
	Pt1.y = image.rows - y_centre;	//Centre du point (y) Les reperes sont differents (BAS =0 sur ROS et HAUT =0 sur openCv)
// 	if(numero_noeud == 0)	//debug
// 		cv::circle(image,Pt1,size_pix/4,cv::Scalar(255,0,0), -1 );
// 	else if(numero_noeud == 213)	//debug
// 		cv::circle(image,Pt1,size_pix/5,cv::Scalar(0,255,255), -1 );
// 	else
		cv::circle(image,Pt1,size_pix/5,cv::Scalar(0,0,255), -1 );
// 	for(int i = 0 ; i < adjac.size() ; i++){
// 		Pt2.x = adjac[i]->getX();
// 		Pt2.y = adjac[i]->image.rows - adjac[i]->getY();
// 		cv::line(image, Pt1, Pt2,cv::Scalar(255,0,0),5);
// 		
// 	}

}


void Noeud::color_chemin(){
	if(! image.data ){
		std::cout <<  "Could not open or find the image" << std::endl ;
		return ;
	}

	cv::Point Pt1, Pt2;
	Pt1.x = x_centre;	//Centre du cercle (x)
	Pt1.y = image.rows - y_centre;	//Centre du point (y) Les reperes sont differents (BAS =0 sur ROS et HAUT =0 sur openCv)
// 	if(numero_noeud == 0)	//debug
		cv::circle(image,Pt1,size_pix/5,cv::Scalar(255,0,0), -1 );
// 	else if(numero_noeud == 213)	//debug
// 		cv::circle(image,Pt1,size_pix/5,cv::Scalar(0,255,255), -1 );
// 	else
// 		cv::circle(image,Pt1,size_pix/5,cv::Scalar(0,0,255), -1 );
	for(int i = 0 ; i < adjac.size() ; i++){
		Pt2.x = adjac[i]->getX();
		Pt2.y = adjac[i]->image.rows - adjac[i]->getY();
		cv::line(image, Pt1, Pt2,cv::Scalar(0,0,255),5);
		
	}
}


void Noeud::display(){
	cv::namedWindow( "Display_dijkstra", cv::WINDOW_NORMAL );// Create a window for display.
	if (!image.empty()) {
		cv::imshow( "Display_dijkstra", image );                   // Show our image inside it.
	}
	cv::waitKey(0);                                          // Wait for a keystroke in the window
}


int Noeud::getX(){
	return x_centre;
}


int Noeud::getY(){
	return y_centre;
}


bool Noeud::check_adjac(Noeud* Noeud_adj){
	if(abs(Noeud_adj->getX() - x_centre) == size_pix && Noeud_adj->getY() == y_centre){
		adjac.push_back(Noeud_adj);
		return true;
	}
	if(abs(Noeud_adj->getY() - y_centre) == size_pix && Noeud_adj->getX() == x_centre){
		adjac.push_back(Noeud_adj);
		return true;
	}
	return false;
}

bool Noeud::check_adjac_es(Noeud* Noeud_adj){
	if(abs(Noeud_adj->getX() - x_centre) <= size_pix && abs(Noeud_adj->getY() - y_centre) <= size_pix){
		adjac.push_back(Noeud_adj);
		return true;
	}
	return false;
}



std::vector< Noeud* > Noeud::get_adjac(){
	return adjac;
}

Noeud* Noeud::get_antecedent(){
	if(antecedent.size() > 0)
		return antecedent[0];
}

void Noeud::set_antecedent(Noeud* antecedent_){
	if(antecedent.size() > 0)
		antecedent[0] = antecedent_;
	else
		antecedent.push_back(antecedent_);
	
}


bool Noeud::antecedent_exist(){
	if(antecedent.size() > 0)
		return true;
	else
		return false;
}


