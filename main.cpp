#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <list>
#include <iterator>

using namespace std;
//2D Point to simplify notations
struct Point2D
{
    double x;
    double y;
    Point2D(float _x, float _y) : x(_x), y(_y) {}
    Point2D() : x(0.f), y(0.f) {}
   
};

bool operator==(const Point2D& a, const Point2D& b)
{
    return a.x == b.x && a.y == b.y;
}
bool operator!=(const Point2D& a, const Point2D& b)
{
    return a.x != b.x || a.y == b.y;
}
Point2D operator+(const Point2D& a, const Point2D& b)
{
    return Point2D(a.x+b.x, a.y+b.y);
}
Point2D operator+=(Point2D& a, const Point2D& b)
{
    a = a+b;
}
Point2D operator-(const Point2D& a, const Point2D& b)
{
    return Point2D(a.x-b.x, a.y-b.y);
}
Point2D operator*(float k, const Point2D& a)
{
    return Point2D(k*a.x, k*a.y);
}
Point2D operator*=(Point2D& a, float k)
{
    a = k*a;
}
//Lap representation as list of checkpoints
struct Lap {
    vector<Point2D> lap;
    int count;
    Lap(vector<Point2D> list): lap(list),count(this->lap.size()){}
    Lap():lap(vector<Point2D>{Point2D()}),count(0){}
};

//Returns distance between two points
float distance(Point2D a,Point2D b) 
{ 
    // Calculating distance 
    Point2D temp = a-b;
    return sqrt(pow(temp.x,2)+pow(temp.y,2)); 
} 
//returns position of max value in 2D vector
vector<int> findMax(vector<vector<double>> vec){
    vector<int> tmp;
    for(int i=0;i<vec.size();i++){
        tmp.insert(tmp.end(),vec[i].begin(),vec[i].end());
    }
    //get the row and column location of the elment
    int row = (max_element(tmp.begin(),tmp.end()) -tmp.begin())/vec.size();
    int col = (max_element(tmp.begin(),tmp.end()) -tmp.begin())%vec.size();
    // gets the value of the max element in O(n) time
    int val = *max_element(tmp.begin(),tmp.end());
    vector<int> maxPos {row,col};
    return maxPos;
}
 //Function to ensure robot is moving only forward
int backwardNoAcceleration(int nextCheckpointAngle){
    int thrust;
    if (nextCheckpointAngle > 90 or nextCheckpointAngle <-90) {
        thrust = 0;
    }
    else thrust = 100;
    return thrust;
}
//Printing out a list of Point2D
void print(vector<Point2D> list){
    cerr <<"Printing Out List:" <<endl;
    for (auto point:list)
    {
        cerr<<"x: " <<point.x << " y: " <<point.y << endl;
    }
    
}

class BoostManager{
    Lap lap;
    Point2D startTriger;
    Point2D endTriger;
    //default constructor
    public :
        BoostManager(){};
    //Constructor
    public:
        BoostManager(Lap lap){
            this->lap = lap;
            boostTrigger(this->lap);
        }

    //Determine if should boost based on longuest distances btw points
    bool boost(Point2D currentCheckpoint){
        if (currentCheckpoint == startTriger) return true;
        return false;
    }
    //find boost triggering checkpoint
    void boostTrigger(Lap Lap){
        vector<vector<double>> distances (lap.count,vector<double>(lap.count,0));
        for ( int i = 0; i < lap.count; i++)
        {
            for ( int j = i+1; j < lap.count; j++)
            {
             distances[i][j] = distance(lap.lap[i],lap.lap[j]) ;
            } 
        }
        vector<int> maxPos = findMax(distances);
        this->startTriger = lap.lap[maxPos[0]];
        this->endTriger = lap.lap[maxPos[1]];
    }

} ;

int main()
{
    vector<Point2D> checkpointList = vector<Point2D>();
    BoostManager bm = BoostManager(Lap(checkpointList));
    bool store = true;
    bool boost = false;

    // game loop
    while (1) {
        int x;
        int y;
        int nextCheckpointX; // x position of the next check point
        int nextCheckpointY; // y position of the next check point
        int nextCheckpointDist; // distance to the next checkpoint
        int nextCheckpointAngle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
        int opponentX;
        int opponentY;
        cin >> opponentX >> opponentY; cin.ignore();

        Point2D newPoint = Point2D(nextCheckpointX,nextCheckpointY);
        //create lap
        if(!checkpointList.empty() && checkpointList[0]!= newPoint && checkpointList[checkpointList.size()-1]!=newPoint &&store) 
            checkpointList.push_back(newPoint);
        else if (store)
        {   store = false;
            //lap known; initialize boost manager
            bm = BoostManager(Lap(checkpointList));
        }
        if (!store) boost = bm.boost(newPoint);
        //boost of accelerate based on case
        if (boost==true) cout << nextCheckpointX << " " << nextCheckpointY << " " << "BOOST" << endl;
        else cout << nextCheckpointX << " " << nextCheckpointY << " " << backwardNoAcceleration(nextCheckpointAngle) << endl;        
    
    }

}

