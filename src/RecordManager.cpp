
#include "RecordManager.h"

#include <iostream>
#include <fstream>


//Utility

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

bool basicSort (int i,int j) { return (i<j); }

//Body of class

std::vector<int> RecordManager::getRecords(){
	
	std::vector<std::string> records;
	std::vector<int> points_and_level;
	
	
	//Opens file and get all ifnfo
	std::string line;
	std::ifstream myfile ("records.txt");
	if (myfile.is_open()){
		while ( getline (myfile,line) ){
			records.push_back(line);
		}
		myfile.close();
	}
	
	//Gets top level with points
	int bestLevel = -999;
	int iteratorNumber;

	for(unsigned int i=0; i<records.size(); i++){
		int levelReaded = atoi((split(records.at(i),':')[1]).c_str());
		if(bestLevel < levelReaded){
			bestLevel = levelReaded;
			iteratorNumber = i;
		}
	}
	
	if(bestLevel != -999){
		int points = atoi((split(records.at(iteratorNumber),':')[0]).c_str());
		points_and_level.push_back(points);
		points_and_level.push_back(bestLevel);
	}

	
	return points_and_level;
	
}

void RecordManager::keepRecord(int points, int level){
	
	std::ofstream myfile ("records.txt", std::ios::app);
	if (myfile.is_open()){
		myfile << points << ":" << level << std::endl;
		myfile.close();
	}
	else std::cout << "Unable to open file";
	
}


RecordManager::RecordManager(){
	_points = 0;
}

int RecordManager::getPoints(){
	return _points;
}

void RecordManager::addPoints(int points){
	_points += points;
}






