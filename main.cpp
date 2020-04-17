#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <list>
#include <iterator>

using namespace std;
#define pi 3.14159265
#define max_speed 100
#define max_oversee_ahead 2000
#define max_avoid_force 100
#define radius_pot 400 * 2



const int angleToSteer = 1;
const int angleToBoost = 5;
const int  maxThrust = 100;
const int angleSlowDown = 90;
const int k = 2;
const float slowDownRadius = 600*k;
const int rotationFactor = 4;


//2D Point to simplify notations
struct Vector2D
{
    double x;
    double y;
    Vector2D(float _x, float _y) : x(_x), y(_y) {}
    Vector2D() : x(0.f), y(0.f) {}
   
};
//Circle data type
struct Circle {
    double r;//radius
    Vector2D center;//center point
    Circle(Vector2D _center,float _r):center(_center),r(_r){};
    //empty circle at the center(0,0)
    Circle():center(Vector2D(0.0,0.0)),r(0.0){};

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
        if (currentCheckpoint == endTriger) {
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
        cerr<<"Boost Triggered at checkpoint";
        cerr<<maxElementIndex+1<<endl;
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

//Collision Avoidance 
//https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
//Fint if the line intersect the circle defined by its radius and center 

bool lineIntersectedCircle(Vector2D ahead,Vector2D ahead2,Circle obstacle){
    
    bool result = distance(obstacle.center,ahead) <= obstacle.r || distance(obstacle.center,ahead2)<=obstacle.r;
    cerr<<"Collision: "<<result<<endl;
    return result;
}

Vector2D collisionAvoidance(Vector2D currentPos,Vector2D opponentPos,Vector2D velocity){
    Vector2D avoidance = Vector2D();
    Vector2D ahead = currentPos + normalize(velocity)* max_oversee_ahead;
    float dynamic_length = length(velocity) / max_oversee_ahead;
    ahead*=dynamic_length;
    Vector2D ahead2 = currentPos + normalize(velocity)*max_oversee_ahead * 0.5;

    if (lineIntersectedCircle(ahead,ahead2,Circle(opponentPos,radius_pot))) {
        cerr<<"Collision Happened"<<endl;
        avoidance.x = ahead.x - opponentPos.x;
        avoidance.y = ahead.y - opponentPos.y;

        avoidance = normalize(avoidance);
        avoidance = avoidance * max_avoid_force;
    }
    return avoidance;
}
//Regularize shield Usage, shield allows to have more weights during collisions
bool shield(Vector2D pod,Vector2D opponent,int nextCheckpointAngle,int prevX,int prevY){
    if(nextCheckpointAngle >= -angleToSteer && nextCheckpointAngle<=angleToSteer){
        return distance(pod,opponent) < slowDownRadius;// && dot(opponent-pod,(Vector2D(prevX,prevY)-pod))>k;
    }
    return false;
}

int main()
{
    int prevX = 0;
    int prevY = 0;

 
    
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
        int useShield = false;// if shield should be used or not
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
        //Thrust Management
        Vector2D velocity(nextCheckpointX-x,nextCheckpointY-y);
        velocity = normalize(velocity)*max_speed;
        if(nextCheckpointAngle<= -angleToSteer || nextCheckpointAngle >= angleToSteer){
            
            Vector2D currDir = rotate(velocity,-nextCheckpointAngle);
            currDir = normalize(currDir);
            Vector2D steeringDir = velocity - currDir;
            steeringDir = normalize(steeringDir) * 100;
            //Compensating for seeking
            nextCheckpointX += steeringDir.x;
            nextCheckpointY += steeringDir.x;
            //Slow Down for angle
            if(nextCheckpointAngle<=-angleSlowDown || nextCheckpointAngle>=angleSlowDown){
                thrust = 0;
            }
            else if (nextCheckpointDist < slowDownRadius) {
                    thrust=maxThrust*(angleSlowDown - abs(nextCheckpointAngle))/ float(angleSlowDown);
                }
        }
        else {
            if (useBoost){
                cerr <<"Will Boost"<<endl;
            }
            else if (nextCheckpointDist <slowDownRadius) {
                //Slow down for radius
                thrust = maxThrust* nextCheckpointDist / slowDownRadius;
            }
        }
        //Rotate more smoothly using pos's speed
        int vx = x - prevX;
        int vy = y - prevY;
        nextCheckpointX+=-rotationFactor*vx;
        nextCheckpointY+=-rotationFactor*vy;
        //Boosting
        if (nextCheckpointAngle>=-angleToBoost && nextCheckpointAngle<= angleToBoost) {
            useBoost = bm.boost(newPoint) && (!store);
            useShield = (shield(Vector2D(x,y),Vector2D(opponentX,opponentY),nextCheckpointAngle,prevX,prevY));
        }
        //Obstacle Avoidance
        Vector2D avoidanceForce = collisionAvoidance(Vector2D(x,y),Vector2D(opponentX,opponentY),velocity);
        cerr<<"Opponent: X "<<opponentX<<" Y "<<opponentY<<endl;
        cerr<<"Avoidance: "<<avoidanceForce.x<<" "<<avoidanceForce.y<<endl;
        cerr<<"Pos:"<<"X "<<x<<" Y "<<y<<endl;
        cerr<<"Shield: "<<useShield<<endl;
        cerr<<"Boost: "<<useBoost<<endl;
        nextCheckpointX+=avoidanceForce.x;
        nextCheckpointY+=avoidanceForce.y;
        //cout
        cout<<nextCheckpointX<<" "<<nextCheckpointY<<" ";
        if(useBoost){
            cout<<"BOOST"<<endl;
        }
        else if (useShield){
            cout<<"SHIELD"<<endl;
        }
        else {
            cout<<thrust<<endl;
        }
        prevX = x;
        prevY = y;
    
    }

}
