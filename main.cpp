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
int main()
{
    int laps;
    cin >> laps; cin.ignore();
    int checkpointCount;
    cin >> checkpointCount; cin.ignore();
    
    vector<Vector2D> checkpointList ; //list of checkpoints


    for (int i = 0; i < checkpointCount; i++) {
        int checkpointX;
        int checkpointY;
        cin >> checkpointX >> checkpointY; cin.ignore();
        cerr<<checkpointX<<" "<<checkpointY<<endl;
        checkpointList.push_back(Vector2D(checkpointX,checkpointY));
        
    }
        cerr<<checkpointList[0].x<<" "<<checkpointList[0].y<<endl;
    // game loop
    while (1) {
        
        vector<int> nextCheckpointIdPods ;
        vector<int> nextCheckpointIdOpps;
        vector<Vector2D> pods;//list of current position of pods
        vector<Vector2D> speedPods;//list of speed of pods
        vector<int> anglePods;
        vector<Vector2D> opps;//position of opponents
        vector<Vector2D> speedOpps;//list of speed of opponents
        vector<int> angleOpps;
        
        for (int i = 0; i < 2; i++) {
            int x; // x position of your pod
            int y; // y position of your pod
            int vx; // x speed of your pod
            int vy; // y speed of your pod
            int angle; // angle of your pod
            int nextCheckPointId; // next check point id of your pod
            cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();
            nextCheckpointIdPods.push_back(nextCheckPointId);
            pods.push_back(Vector2D(x,y));
            speedPods.push_back(Vector2D(vx,vy));
            anglePods.push_back(angle);

        }
        for (int i = 0; i < 2; i++) {
            int x2; // x position of the opponent's pod
            int y2; // y position of the opponent's pod
            int vx2; // x speed of the opponent's pod
            int vy2; // y speed of the opponent's pod
            int angle2; // angle of the opponent's pod
            int nextCheckPointId2; // next check point id of the opponent's pod
            cin >> x2 >> y2 >> vx2 >> vy2 >> angle2 >> nextCheckPointId2; cin.ignore();
            nextCheckpointIdOpps.push_back(nextCheckPointId2);
            opps.push_back(Vector2D(x2,y2));
            speedOpps.push_back(Vector2D(vx2,vy2));
            angleOpps.push_back(angle2);
        }
        
        cout <<checkpointList[nextCheckpointIdPods[0]].x<<" "<<checkpointList[nextCheckpointIdPods[0]].y<<" 100"<< endl;
        cout <<checkpointList[nextCheckpointIdPods[1]].x<<" "<<checkpointList[nextCheckpointIdPods[1]].y<<" 100"<< endl;
    }
}