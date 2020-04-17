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
const int k = 25;
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
//A class to manage boost usage
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
            //this->boostTrigger();
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
        if (this->lap.count < 2) {
            cerr<<"All checkpoints not considered"<<endl;
        }
        for ( int i = 0; i <this-> lap.count; i++)
        {
            distances[i] = distance(this->lap.lap[i],this->lap.lap[((i+1)==lap.count) ? 0 : i+1]) ;
        }
        int maxElementIndex = std::max_element(distances.begin(),distances.end()) - distances.begin();
        this->startTriger = this->lap.lap[maxElementIndex];
        int endElementIndex =((maxElementIndex+1)==0)? 0:maxElementIndex+1;
        this->endTriger = this->lap.lap[endElementIndex];
        cerr<<"Boost will be Triggered betwen checkpoint ";
        cerr<<maxElementIndex<<endl;
    }

} ;
//find realtive angle between target and pod
float relativeAngle (int angle,Vector2D target,Vector2D pod) {
    Vector2D dir = normalize((target - pod));
    float res = acos(dir.x) * 180.0f / pi;
    if (dir.y < 0.f) return 360.f - res;
    return res;
}
//Pod simplication
class Pod
{
    public:
        Vector2D pos;
        Vector2D speed;
        int absAngle;
        int checkpointId;
        bool boostAvailable;
        Vector2D nexCheckpoint;
        int thrust;
        bool useBoost;

    public:
        Pod() : pos(0, 0), speed(0, 0), absAngle(0), checkpointId(0), boostAvailable(true), nexCheckpoint(0, 0), thrust(maxThrust),useBoost(false){};
        const Vector2D& Position() const { return pos; }
        const Vector2D& Speed() const { return speed; }
        const int Angle()  const { return absAngle; }
        const int CheckpointId() const { return checkpointId; }
        int  getThrust() const { return thrust; }
        const Vector2D& NextCheckpoint() const { return nexCheckpoint; }
        void SetThrust(int _thrust) { thrust = _thrust; }
        void SetnextCheckpoint(const Vector2D& _nexCheckpoint) { nexCheckpoint = _nexCheckpoint; }
        void setBoostAvailable(const int _boostAvailable) { boostAvailable = _boostAvailable ;}
        bool boost()
        {
            if (!boostAvailable) return false;
            useBoost = true;
            boostAvailable = false;
            return useBoost && boostAvailable;
        }
        void init(int x,int y,int vx,int vy,int angle,int nextCheckPointId) {
            this->pos = Vector2D(x,y);
            this->speed = Vector2D(vx, vy);
            this->absAngle = angle;
            this->checkpointId = nextCheckPointId;
            this->useBoost = false;
        }
};

int findThrust(Pod& pod, int checkpointToBoost) {
    const Vector2D target = pod.NextCheckpoint();
    int thrust = maxThrust;
    Vector2D velocity(target-pod.pos);
    velocity = normalize(velocity)*max_speed;
    const float nextCheckpointDist = distance(target, pod.Position());
    const float relAngle = relativeAngle(pod.Angle(),target,pod.Position()) - pod.Angle();
    if(relAngle<= -angleToSteer || relAngle >= angleToSteer){
            
        Vector2D currDir = rotate(velocity,-relAngle);
        currDir = normalize(currDir);
        Vector2D steeringDir = velocity - currDir;
        steeringDir = normalize(steeringDir) * 100;
        //Compensating for seeking
        pod.SetnextCheckpoint(pod.NextCheckpoint()+steeringDir);
        if(relAngle<=-angleSlowDown || relAngle>=angleSlowDown){
            thrust = 0;
        }
        else if (nextCheckpointDist < slowDownRadius) {
                thrust=maxThrust*(angleSlowDown - abs(relAngle))/ float(angleSlowDown);
            }
    }
    else {
        if (nextCheckpointDist <slowDownRadius) {
            //Slow down for radius
            thrust = maxThrust* nextCheckpointDist / slowDownRadius;
        }
    }
    // if (pod.Angle()<angleToSteer) {
    //     thrust = maxThrust;    
    // }
    // else {
    //     // try to avoid drifts
    //     pod.SetnextCheckpoint(target - float(k * pod.Speed()));
    //     // in order to align, we also need to reduce the thrust
    //     const float dist = distance(target, pod.Position());
    //     const float distanceSlowdownFactor = clamp(dist/(slowDownRadius), 0.f, 1.f);
    //     const float angleSlowdownFactor = 1.f - clamp(abs(relAngle)/90.f, 0.f, 1.f);
    //     cerr << "Slowdown: distance " << distanceSlowdownFactor << " - angle " << angleSlowdownFactor << endl;
    //     thrust -= maxThrust * distanceSlowdownFactor * angleSlowdownFactor;
    // }
    return thrust;
}

int main()
{
    int laps;
    cin >> laps; cin.ignore();
    int checkpointCount;
    cin >> checkpointCount; cin.ignore();
    
    vector<Vector2D> checkpointList ; //list of checkpoints
    BoostManager bm;
    for (int i = 0; i < checkpointCount; i++) {
        int checkpointX;
        int checkpointY;
        cin >> checkpointX >> checkpointY; cin.ignore();
        checkpointList.push_back(Vector2D(checkpointX,checkpointY));  
    }
    //initialize bm
    bm = BoostManager(Lap(checkpointList));
    
    vector<Pod> pods = vector<Pod>(checkpointCount);
    vector<Pod> opps = vector<Pod>(checkpointCount);
    // game loop
    while (1) {
        for (int i = 0; i < 2; i++) {
            int x; // x position of your pod
            int y; // y position of your pod
            int vx; // x speed of your pod
            int vy; // y speed of your pod
            int angle; // angle of your pod
            int nextCheckPointId; // next check point id of your pod
            cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();
            pods[i].init(x,y,vx,vy,angle,nextCheckPointId);
        }
      
        //set next checkpoint,check boost and computer thrust for pods
        for (int i= 0; i< 2;i++){
            pods[i].SetnextCheckpoint(checkpointList[pods[i].CheckpointId()]);//set next checkpoint
            if (pods[i].Angle() >=-angleToBoost && pods[i].Angle()<=angleToBoost) {
                pods[i].setBoostAvailable(bm.boost(pods[i].NextCheckpoint()));//find thrust
                }//check boost
            pods[i].SetThrust(findThrust(pods[i],pods[i].CheckpointId()));

         }
        
        //outputs
        cout << pods[0].NextCheckpoint().x << " " << pods[0].NextCheckpoint().y << " ";
        if (pods[0].boost()) cout << "BOOST";
        else cout << pods[0].getThrust();
        cout<<endl;
        cout<<"0 0 100"<<endl;
    }
}
