#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <list>
#include <iterator>

using namespace std;
#define pi 3.14159265

//2D Point to simplify notations
struct Vector2D
{
    double x;
    double y;
    Vector2D(float _x, float _y) : x(_x), y(_y) {}
    Vector2D() : x(0.f), y(0.f) {}
   
};

bool operator==(const Vector2D& a, const Vector2D& b)
{
    return a.x == b.x && a.y == b.y;
}
bool operator!=(const Vector2D& a, const Vector2D& b)
{
    return a.x != b.x || a.y != b.y;
}
Vector2D operator+(const Vector2D& a, const Vector2D& b)
{
    return Vector2D(a.x+b.x, a.y+b.y);
}
Vector2D operator+=(Vector2D& a, const Vector2D& b)
{
    a = a+b;
}
Vector2D operator-(const Vector2D& a, const Vector2D& b)
{
    return Vector2D(a.x-b.x, a.y-b.y);
}
Vector2D operator*(float k, const Vector2D& a)
{
    return Vector2D(k*a.x, k*a.y);
}
Vector2D operator*=(Vector2D& a, float k)
{
    a = k*a;
}
Vector2D operator * (Vector2D a, const float s ) {
     return Vector2D( s*a.x, s*a.y ); 
}

Vector2D operator / (Vector2D& a, const float s ) {
    	float r = float(1.0) / s;
	    return a * r;
}
float dot( const Vector2D& u, const Vector2D& v ) {
    return u.x * v.x + u.y * v.y;
}
float length( const Vector2D& v ) {
    return sqrt( dot(v,v) );
}
Vector2D normalize( const Vector2D& v ) {
    return v * float(1.0/length(v));
}

inline
Vector2D rotate( const Vector2D& v, float angle ) {
    float radian = angle * pi / 180;
    double sinAngle = sin(radian);
    double cosAngle = cos(radian);
    
    return Vector2D( v.x * cosAngle - v.y * sinAngle, v.y * cosAngle + v.x * sinAngle );
}
//Lap representation as list of checkpoints
struct Lap {
    vector<Vector2D> lap;
    int count;
    Lap(vector<Vector2D> list): lap(list),count(list.size()){}
    Lap():lap(vector<Vector2D>{Vector2D()}),count(0){}
};

//Returns distance between two points
float distance(Vector2D a,Vector2D b) 
{ 
    // Calculating distance 
    Vector2D temp = a-b;
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
//Printing out a list of Vector2D
void print(vector<Vector2D> list){
    cerr <<"Printing Out List:" <<endl;
    for (auto point:list)
    {
        cerr<<"x: " <<point.x << " y: " <<point.y << endl;
    }
    
}

class BoostManager{
    Lap lap;
    Vector2D startTriger;
    Vector2D endTriger;
    //default constructor
    public :
        BoostManager(){
        
        };
    //Constructor
    public:
        BoostManager(Lap lap){
            this->lap = lap;
        }
    private:
        bool boostAvailable = true;
    public: 
    //Determine if should boost based on longuest distances btw points
    bool boost(Vector2D currentCheckpoint){
        int boost;
        if (currentCheckpoint == startTriger) {
            boost = true && this->boostAvailable ;
            this->boostAvailable = false;
        }
        else boost = false;
        return boost;
    }
    //find boost triggering checkpoint
    void boostTrigger(){
        vector<double> distances = vector<double>(this->lap.count,0);
        for ( int i = 0; i <this-> lap.count; i++)
        {
            distances[i] = distance(this->lap.lap[i],this->lap.lap[((i+1)==lap.count) ? 0 : i+1]) ;
        }
        int maxElementIndex = std::max_element(distances.begin(),distances.end()) - distances.begin();
        this->startTriger = this->lap.lap[maxElementIndex];
        this->endTriger = this->lap.lap[((maxElementIndex+1)==0)? 0:maxElementIndex+1];
        cerr<<"Trigger"<<endl;
        cerr<<maxElementIndex<<" "<<endl;
    }

} ;
class ThurstManager{
    int maxThrust = 100;
    int epsilon = 2;//small angle
    int k = 2;//a factor
    int r_checkpoint = 600;//raduis of checkpoint
    public :
        ThurstManager(){};
    //compute required thrust based on angle and distance    
    int requiredThrust(Vector2D nextCheckpoint,int nextCheckpointAngle){
        cerr<<nextCheckpointAngle<<endl;
        if (nextCheckpointAngle >= 90 || nextCheckpointAngle <=-90) return 0;
        else if (abs(nextCheckpointAngle) <= this->epsilon) return maxThrust;
        else return maxThrust*(1- abs(nextCheckpointAngle)/90);//*sqrt(pow(nextCheckpoint.x,2)+pow(nextCheckpoint.y,2))/k*r_checkpoint;
    }

};

int main()
{

    int prevX = 0;
    int prevY = 0;

    const int angleToSteer = 1;
    const int angleSlowDown = 90;
    const float slowDownRadius = 600*4;

    vector<Vector2D> checkpointList = vector<Vector2D>();
    BoostManager bm;
    ThurstManager tm;
    bool store = true;
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

        int thrust = 100;//initialize acceleration to its max
        int useBoost  = false ;//know if should use boost or not
        
        Vector2D newPoint = Vector2D(nextCheckpointX,nextCheckpointY);
        //create lap
        if (checkpointList.empty()){
            checkpointList.push_back(newPoint);
        } 
        else if(checkpointList[0]!=newPoint && store && checkpointList[checkpointList.size()-1]!=newPoint) {
            checkpointList.push_back(newPoint);
        }
        else if (checkpointList[0]==newPoint && checkpointList[checkpointList.size()-1]!=newPoint && store) {   
            cerr<<"Trigger"<<endl;
            store = false;
            //lap known; initialize boost manager
            bm = BoostManager(Lap(checkpointList));
            bm.boostTrigger();
        }
        //find if should boost
        if (!store) useBoost = bm.boost(newPoint);
        
        if(nextCheckpointAngle<= -angleToSteer || nextCheckpointAngle >= angleToSteer){
            //1. Seeking
            Vector2D desiredDir(nextCheckpointX-x,nextCheckpointY-y);
            desiredDir= normalize(desiredDir);
            Vector2D currDir = rotate(desiredDir,-nextCheckpointAngle);
            currDir = normalize(currDir);
            Vector2D steeringDir = desiredDir - currDir;
            steeringDir = normalize(steeringDir) * 100;
            //Compensating
            nextCheckpointX += steeringDir.x;
            nextCheckpointY += steeringDir.x;

            //Slow Down for angle
            if(nextCheckpointAngle<=-angleSlowDown || nextCheckpointAngle>=angleSlowDown){
                thrust = 0;
            }
            else if (nextCheckpointDist < slowDownRadius) {
                thrust*=(angleSlowDown - abs(nextCheckpointAngle))/ float(angleSlowDown);
            }
        }
        else {
            if (useBoost){
                cerr <<"Will Boost"<<endl;
            }
            else if (nextCheckpointDist <slowDownRadius) {
                //Slow down for radius
                thrust*= nextCheckpointDist / slowDownRadius;
            }
        }
        
        //cout
        cout<<nextCheckpointX<<" "<<nextCheckpointY<<" ";
        if(useBoost){
            cout<<"BOOST"<<endl;
        }
        else {
            cout<<thrust<<endl;
        }
    
    }

}
