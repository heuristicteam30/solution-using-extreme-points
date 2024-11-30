//#include "bits/stdc++.h"
#include "bits/stdc++.h"
#include "solver.h"
//#include "genetic.cpp"
#define int long long
#define For(i,n) for(int i=0; i<n;i++)
#define FOR(k,i,n) for(int i=k; i<n;i++)
#define vi vector<int>
#define max(a,b) (a>b?a:b)
#define maxP(a,b) (a.first>b.first?a:b)
#define min(a,b) (a<b?a:b)
#define INF 10000000000000000
#define pii pair<int,int>
#define NON_PRIORITY_COST 1000000
#define PRIORITY_ULD_COST 5000
#define RESIDUE_THRESHOLD 0
#define convertCoords(pt) pair<int,pair<int,pii>>(pt.box,pair<int,pii>(pt.x,pii(pt.y,pt.z)))
// class Solver;
// struct Box{
//     int l,b,h,weight,cost,ID;bool isPriority;
// };
// struct Sorter {
// //    function<void(Solver*)>init;
//     function<bool(Box,Box)>val;
// };
// struct coords{
//     int x,y,z,box;
// };
// struct Merit {
// //    function<void(Solver*)>init;
//     function<int(coords,Box,Solver*)>val;
// };
// struct Uld {
//     Box dim;
//     int weight, maxWt, ID;
//     coords com, maxBound;
// };
// class Solver{
// public:
//     vi meritVar, sortVar;
//     map<pair<int,pair<int,pii>>,pair<int,pii>>ep;//(box,(x,(y,z))):(x,(y,z))
//     Merit merit;Sorter sorter;
//     vector<pair<coords,Box>>placement;//(bottom left corner, dimensions)
//     vector<set<int>>ULDPackages;//sees which package is placed in which ULD
//     vector<bool>ULDHasPriority;
//     vector<Box>data;
//     coords def;
//     vector<Uld>ULDl;
//     Solver(Sorter sorter_, Merit merit_, vector<Box> boxes,vector<Uld> ULD_){
//         this->merit = merit_;
//         this->sorter = sorter_;
//         //        this->merit.init(this);
//         //        this->sorter.init(this);
//         this->data = boxes;
//         def.x = def.y = def.z=def.box  -1;
//         Box def_;def_.l = def_.b = def_.h=  -1;
//         this->placement.assign(boxes.size(), pair<coords,Box>(def,def_));
//         ULDl = ULD_;
//         ULDPackages.assign(ULDl.size(), set<int>());
//         ULDHasPriority.assign(ULDl.size(), false);
//     }
//     int cost(){
//         int c=0;
//         set<int>priorityShipments;
//         For(i,data.size()){
//             if(placement[i].first.x==-1){
//                 c-=data[i].cost;
//                 if(data[i].isPriority){
//                     c-=NON_PRIORITY_COST;
//                 }
//             }else{
// //                c+=data[i].cost;
                
//                 if(data[i].isPriority)priorityShipments.insert(placement[i].first.box);
//             }
//         }
//         c-=priorityShipments.size()*PRIORITY_ULD_COST;
// //        For(i,ULDl.size())c-=abs(ULDl[i].dim.l/2-ULDl[i].com.x)+abs(ULDl[i].dim.b/2-ULDl[i].com.y)+abs(ULDl[i].dim.h/2-ULDl[i].com.z);
// //        cout<<c<<"\n";
//         return c;
//     }
//     bool checkCollision(coords e, Box b){
//         //checks collision of prespective packages with all other packages of sam eULD
//         if(e.x+b.l>=ULDl[e.box].dim.l or e.y+b.b>=ULDl[e.box].dim.b or e.z+b.h>=ULDl[e.box].dim.h or ULDl[e.box].weight+b.weight>ULDl[e.box].maxWt)return true;
//         for(auto i:ULDPackages[e.box]){
//             auto x = placement[i];
//             if((x.first.x<b.l+e.x and x.first.y<b.b+e.y and x.first.z<b.h+e.z and e.x<x.first.x+x.second.l and e.y<x.first.y+x.second.b and e.z<x.first.z+x.second.h)){return true;}
//         }
//         return false;
//     }
//     void solve(){
//         sort(data.begin(),data.end(),this->sorter.val);
//         For(i,ULDl.size())ep[pair<int,pair<int,pii>>(i,pair<int, pii>(0,pii(0,0)))] = pair<int, pii>(ULDl[i].dim.l,pii(ULDl[i].dim.b,ULDl[i].dim.h));
//         For(i,data.size()){
//             Box b = data[i];
//             pair<int,pair<coords,Box>>best;best.first=-INF;
//             vector<Box>perms(6);
//             perms[0].l = b.l;
//             perms[0].b = b.b;
//             perms[0].h = b.h;
//             perms[1].l = b.l;
//             perms[1].h = b.b;
//             perms[1].b = b.h;
//             perms[2].l = b.b;
//             perms[2].b = b.l;
//             perms[2].h = b.h;
//             perms[3].l = b.b;
//             perms[3].b = b.h;
//             perms[3].h = b.l;
//             perms[4].l = b.h;
//             perms[4].b = b.b;
//             perms[4].h = b.l;
//             perms[5].l = b.h;
//             perms[5].b = b.l;
//             perms[5].h = b.b;
//             for(Box p:perms)for(auto x:ep){
//                 //check if it can fit
//                 coords e;e.box = x.first.first; e.x = x.first.second.first;e.y = x.first.second.second.first;e.z = x.first.second.second.second;
//                 if(checkCollision(e, p))continue;
//                 p.isPriority = b.isPriority;
//                 int score = this->merit.val(e,p,this);
//                 if(best.first<score)best = pair<int,pair<coords,Box>>(score,pair<coords,Box>(e,p));
//             }
//             //update vals
//             if(best.first==-INF)continue;
//             if(data[i].isPriority)ULDHasPriority[best.second.first.box]=true;
//             placement[i] = best.second;
//             ULDPackages[best.second.first.box].insert(i);
//             addEP(i);
// //            for(auto i: ep)cout<<i.first.first<<" "<<i.first.second.first<<" "<<i.first.second.second.first<<" "<<i.first.second.second.second<<"\n";
//             update(i);
//         }
//     }
//     void update(int i){
//         int b = placement[i].first.box;
//         updateMaxBound(i);
//         updateResidue(i);
//         ULDl[b].com.x += (placement[i].first.x+placement[i].second.l/2)*data[i].weight;
//         ULDl[b].com.y += (placement[i].first.y+placement[i].second.b/2)*data[i].weight;
//         ULDl[b].com.z += (placement[i].first.z+placement[i].second.h/2)*data[i].weight;
//         ULDl[b].weight+=data[i].weight;
//     }
//     vector<coords> getCOM(){
//         vector<coords> r(ULDl.size());
//         For(i,r.size()){
//             if(ULDl[i].weight==0){r[i].x = ULDl[i].dim.l/2;r[i].y = ULDl[i].dim.b/2;r[i].z = ULDl[i].dim.h/2;continue;}
//             r[i].x = ULDl[i].com.x/ULDl[i].weight;
//             r[i].y = ULDl[i].com.y/ULDl[i].weight;
//             r[i].z = ULDl[i].com.z/ULDl[i].weight;
//         }
//         return r;
//     }
//     pair<int,pii>getResidueSpace(coords src){
//         pair<int,pii>ret(beamprojectXPos(src).x-src.x,pii(beamprojectYPos(src).y-src.y,beamprojectZPos(src).z-src.z));
//         return ret;
//     }
//     void initialise(coords&X, coords&ob, int x, int y, int z)
//     {
//         X.x=ob.x+x;
//         X.y=ob.y+y;
//         X.z =ob.z+z;
//         X.box = ob.box;
    
//     }
//     coords beamprojectZNeg(coords ob1)
//     {
//         coords p,q,r;
//         initialise(p,ob1,1,0,0);
//         initialise(q,ob1,0,1,0);
//         initialise(r,ob1,1,1,0);
//         coords p1=rayProjectZNeg(p);
//         coords q1=rayProjectZNeg(ob1);
//         coords r1=rayProjectZNeg(q);
//         coords s1=rayProjectZNeg(r);
//         coords ans;
//         ans.x=ob1.x;
//         ans.y=ob1.y;
//         ans.z=min(min(min(p1.z,q1.z),r1.z),s1.z);
//         ans.box = ob1.box;
//         return ans;
//     }
//     coords beamprojectYNeg(coords ob1)
//     {
//         coords p,q,r;
//         initialise(p,ob1,1,0,0);
//         initialise(q,ob1,0,0,1);
//         initialise(r,ob1,1,0,1);
//         coords p1=rayProjectYNeg(p);
//         coords q1=rayProjectYNeg(ob1);
//         coords r1=rayProjectYNeg(q);
//         coords s1=rayProjectYNeg(r);
//         coords ans;
//         ans.x=ob1.x;
//         ans.y=min(min(min(p1.y,q1.y),r1.y),s1.y);
//         ans.z=ob1.z;
//         ans.box = ob1.box;
//         return ans;
//     }
//     coords beamprojectXNeg(coords ob1)
//     {
//         coords p,q,r;
//         initialise(p,ob1,0,1,0);
//         initialise(q,ob1,0,0,1);
//         initialise(r,ob1,0,1,1);
//         coords p1=rayProjectXNeg(p);
//         coords q1=rayProjectXNeg(ob1);
//         coords r1=rayProjectXNeg(q);
//         coords s1=rayProjectXNeg(r);
//         coords ans;
//         ans.x=min(min(min(p1.x,q1.x),r1.x),s1.x);
//         ans.y=ob1.y;
//         ans.z=ob1.z;
//         ans.box = ob1.box;
//         return ans;
//     }
//     coords beamprojectZPos(coords ob1)
//     {
//         coords p,q,r;
//         initialise(p,ob1,1,0,0);
//         initialise(q,ob1,0,1,0);
//         initialise(r,ob1,1,1,0);
//         coords p1=rayProjectZPos(p);
//         coords q1=rayProjectZPos(ob1);
//         coords r1=rayProjectZPos(q);
//         coords s1=rayProjectZPos(r);
//         coords ans;
//         ans.x=ob1.x;
//         ans.y=ob1.y;
//         ans.z=min(min(min(p1.z,q1.z),r1.z),s1.z);
//         ans.box = ob1.box;
//         return ans;
//     }
//     coords beamprojectYPos(coords ob1)
//     {
//         coords p,q,r;
//         initialise(p,ob1,1,0,0);
//         initialise(q,ob1,0,0,1);
//         initialise(r,ob1,1,0,1);
//         coords p1=rayProjectYPos(p);
//         coords q1=rayProjectYPos(ob1);
//         coords r1=rayProjectYPos(q);
//         coords s1=rayProjectYPos(r);
//         coords ans;
//         ans.x=ob1.x;
//         ans.y=min(min(min(p1.y,q1.y),r1.y),s1.y);
//         ans.z=ob1.z;
//         ans.box = ob1.box;
//         return ans;
//     }
//     coords beamprojectXPos(coords ob1)
//     {
//         coords p,q,r;
//         initialise(p,ob1,0,1,0);
//         initialise(q,ob1,0,0,1);
//         initialise(r,ob1,0,1,1);
//         coords p1=rayProjectXPos(p);
//         coords q1=rayProjectXPos(ob1);
//         coords r1=rayProjectXPos(q);
//         coords s1=rayProjectXPos(r);
//         coords ans;
//         ans.x=min(min(min(p1.x,q1.x),r1.x),s1.x);
//         ans.y=ob1.y;
//         ans.z=ob1.z;
//         ans.box = ob1.box;
//         return ans;
//     }
//     void addEP(int i){
//         ep.erase(pair<int,pair<int,pii>>(placement[i].first.box,pair<int,pii>(placement[i].first.x,pii(placement[i].first.y,placement[i].first.z))));
//         coords ob1;
//         auto p = placement[i];
//         placement[i].first=def;
//         ob1.box = p.first.box;
//         ob1.x=p.first.x;
//         ob1.y=p.first.y+placement[i].second.b;
//         ob1.z=p.first.z+p.second.h;
//         auto t =beamprojectYNeg(ob1);
//         auto r =getResidueSpace(t);
//         if(r.first>RESIDUE_THRESHOLD and r.second.first>RESIDUE_THRESHOLD and  r.second.second>RESIDUE_THRESHOLD)ep[convertCoords(t)] = r;
//         t=beamprojectZNeg(ob1);
//         r =getResidueSpace(t);
//         if(r.first>RESIDUE_THRESHOLD and r.second.first>RESIDUE_THRESHOLD and  r.second.second>RESIDUE_THRESHOLD)ep[convertCoords(t)] = r;
//         coords ob2;
//         ob2.box = p.first.box;
//         ob2.x=p.first.x+p.second.l;
//         ob2.y=p.first.y;
//         ob2.z=p.first.z+p.second.h;
//         t =beamprojectXNeg(ob2);
//         r =getResidueSpace(t);
//         if(r.first>RESIDUE_THRESHOLD and r.second.first>RESIDUE_THRESHOLD and  r.second.second>RESIDUE_THRESHOLD)ep[convertCoords(t)] = r;
//         t =beamprojectZNeg(ob2);
//         r =getResidueSpace(t);
//         if(r.first>RESIDUE_THRESHOLD and r.second.first>RESIDUE_THRESHOLD and  r.second.second>RESIDUE_THRESHOLD)ep[convertCoords(t)] = r;
//         coords ob3;
//         ob3.box = p.first.box;
//         ob3.x=p.first.x+p.second.l;
//         ob3.y=p.first.y+p.second.b;
//         ob3.z=p.first.z;
//         t =beamprojectXNeg(ob3);
//         r =getResidueSpace(t);
//         if(r.first>RESIDUE_THRESHOLD and r.second.first>RESIDUE_THRESHOLD and  r.second.second>=RESIDUE_THRESHOLD)ep[convertCoords(t)] = r;
//         t =beamprojectYNeg(ob3);
//         r =getResidueSpace(t);
//         if(r.first>RESIDUE_THRESHOLD and r.second.first>RESIDUE_THRESHOLD and  r.second.second>RESIDUE_THRESHOLD)ep[convertCoords(t)] = r;
//         placement[i] = p;
//     }
//     void updateMaxBound(int i){
//         int b= placement[i].first.box;
//         ULDl[b].maxBound.x = max(ULDl[b].maxBound.x,placement[i].first.x+placement[i].second.l);
//         ULDl[b].maxBound.y = max(ULDl[b].maxBound.y,placement[i].first.y+placement[i].second.b);
//         ULDl[b].maxBound.z = max(ULDl[b].maxBound.z,placement[i].first.z+placement[i].second.h);
//     }
//     void updateResidue(int i){
//         //update residual space also to be done
//         auto it = ep.lower_bound(convertCoords(placement[i].first));
//         int b = placement[i].first.box;
//         while(it!=ep.end() and it->first.first==b){
//             auto p = *it;
//             coords e;
//             e.box = p.first.first;
//             e.x = p.first.second.first;
//             e.y = p.first.second.second.first;
//             e.z= p.first.second.second.second;
//             int t =XBeamIntersectionWithBox(e,i);
//             if(t!=-1)p.second.first = min(p.second.first,t);
//             t =YBeamIntersectionWithBox(e,i);
//             if(t!=-1)p.second.second.first = min(p.second.second.first,t);
//             t =ZBeamIntersectionWithBox(e,i);
//             if(t!=-1)p.second.second.second = min(p.second.second.second,t);
//             it++;
//         }
//     }
//     int XBeamIntersectionWithBox(coords start,int ind){
//         int r = INF;
//         int b = start.box;
//         For(i,2)For(j,2){
//             auto p = start;p.y = min(p.y+i, ULDl[b].dim.b-1);p.z = min(p.z+j, ULDl[b].dim.h-1);
//             int t = XRayIntersectionWithBox(p,ind);
//             if(t!=-1){
//                 r = min(r,t);
//             }
//         }
//         return r==INF?-1:r;
//     }
//     int YBeamIntersectionWithBox(coords start,int ind){
//         int r = INF;
//         int b = start.box;
//         For(i,2)For(j,2){
//             auto p = start;p.x = min(p.x+i, ULDl[b].dim.l-1);p.z = min(p.z+j, ULDl[b].dim.h-1);
//             int t = YRayIntersectionWithBox(p,ind);
//             if(t!=-1){
//                 r = min(r,t);
//             }
//         }
//         return r==INF?-1:r;
//     }
//     int ZBeamIntersectionWithBox(coords start,int ind){
//         int r = INF;
//         int b = start.box;
//         For(i,2)For(j,2){
//             auto p = start;p.x = min(p.x+i, ULDl[b].dim.l-1);p.y = min(p.y+j, ULDl[b].dim.b-1);
//             int t = ZRayIntersectionWithBox(p,ind);
//             if(t!=-1){
//                 r = min(r,t);
//             }
//         }
//         return r==INF?-1:r;
//     }
//     int XRayIntersectionWithBox(coords start,int ind){
//         if(start.y<placement[ind].first.y+placement[ind].second.b and start.y>placement[ind].first.y and start.z<placement[ind].first.z+placement[ind].second.h and start.z>placement[ind].first.z and start.x<placement[ind].first.x+placement[ind].second.l and start.box == placement[ind].first.box)return placement[ind].first.x;
//         return -1;
//     }
//     int YRayIntersectionWithBox(coords start,int ind){
//         if(start.x<placement[ind].first.x+placement[ind].second.l and start.x>placement[ind].first.x and start.z<placement[ind].first.z+placement[ind].second.h and start.z>placement[ind].first.z and start.y<placement[ind].first.y+placement[ind].second.b and start.box == placement[ind].first.box)return placement[ind].first.y;
//         return -1;
//     }
//     int ZRayIntersectionWithBox(coords start,int ind){
//         if(start.x<placement[ind].first.x+placement[ind].second.l and start.x>placement[ind].first.x and start.y<placement[ind].first.y+placement[ind].second.b and start.y>placement[ind].first.y and start.z<placement[ind].first.z+placement[ind].second.h and start.box == placement[ind].first.box)return placement[ind].first.z;
//         return -1;
//     }
//     coords rayProjectXNeg(coords start){
//         int x = 0;
//         for(int i : ULDPackages[start.box]){
//             if(placement[i].first.x!=-1 and start.y<=placement[i].first.y+placement[i].second.b and start.y>=placement[i].first.y and start.z<=placement[i].first.z+placement[i].second.h and start.z>=placement[i].first.z and start.x>=placement[i].first.x)x = max(x,placement[i].first.x+placement[i].second.l);
//         }
//         start.x = min(start.x,x);
//         return start;
//     }
//     coords rayProjectYNeg(coords start){
//         int x = 0;
//         for(int i : ULDPackages[start.box]){
//             if(placement[i].first.x!=-1 and start.x<=placement[i].first.x+placement[i].second.l and start.x>=placement[i].first.x and start.z<=placement[i].first.z+placement[i].second.h and start.z>=placement[i].first.z and start.y>=placement[i].first.y)x = max(x,placement[i].first.y+placement[i].second.b);
//         }
//         start.y = min(start.y,x);
//         return start;
//     }
//     coords rayProjectZNeg(coords start){
//         int x = 0;
//         for(int i : ULDPackages[start.box]){
//             if(placement[i].first.x!=-1 and start.x<=placement[i].first.x+placement[i].second.l and start.x>=placement[i].first.x and start.y<=placement[i].first.y+placement[i].second.b and start.y>=placement[i].first.y and start.z>=placement[i].first.z)x = max(x,placement[i].first.z+placement[i].second.h);
//         }
//         start.z = min(start.z,x);
//         return start;
//     }
//     coords rayProjectXPos(coords start){
//         int x = ULDl[start.box].dim.l;
//         for(int i : ULDPackages[start.box]){
//             if(placement[i].first.x!=-1 and start.y<placement[i].first.y+placement[i].second.b and start.y>placement[i].first.y and start.z<placement[i].first.z+placement[i].second.h and start.z>placement[i].first.z and start.x<placement[i].first.x+placement[i].second.l)x = min(x,placement[i].first.x);
//         }
//         start.x = max(start.x,x);
//         return start;
//     }
//     coords rayProjectYPos(coords start){
//         int x = ULDl[start.box].dim.b;
//         for(int i : ULDPackages[start.box]){
//             if(placement[i].first.x!=-1 and start.x<placement[i].first.x+placement[i].second.l and start.x>placement[i].first.x and start.z<placement[i].first.z+placement[i].second.h and start.z>placement[i].first.z and start.y<placement[i].first.y+placement[i].second.b)x = min(x,placement[i].first.y);
//         }
//         start.y = max(start.y,x);
//         return start;
//     }
//     coords rayProjectZPos(coords start){
//         int x = ULDl[start.box].dim.h;
//         for(int i : ULDPackages[start.box]){
//             if(placement[i].first.x!=-1 and start.x<placement[i].first.x+placement[i].second.l and start.x>placement[i].first.x and start.y<placement[i].first.y+placement[i].second.b and start.y>placement[i].first.y and start.z<placement[i].first.z+placement[i].second.h)x = min(x,placement[i].first.z);
//         }
//         start.z = max(start.z,x);
//         return start;
//     }
// };
//--------------------------------------------------------------------
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <optional>
#include <numeric>
#include <set>
#include <deque>
#include <string>
#include <sstream>
#include <fstream>

#define NUM_PACKETS 400
#define POPULATION_SIZE 20
#define NUM_GENERATIONS 10
#define WARMSTART_FACTOR 2
#define TOURNAMENT_FACTOR 5
#define ELITISM_PERCENTAGE 0.1
#define MUTATION_PROB 0.5
#define TABU_SIZE 10
#define MAX_TABU_ITERS 20
#define NEIGHBORHOOD_SIZE 10
#define NEIGHBORHOOD_RANGE 3
#define SPREAD_COST 5000
#define BEST_K_SOLNS 3
int residueFunc(coords c, Box b,Solver* s){
    int r= 0;
    r+=s->ULDHasPriority[c.box]*100000000000*b.isPriority;
    float relativeDifference =(s->ep[convertCoords(c)].first - b.l)/1.0/s->ep[convertCoords(c)].first+(s->ep[convertCoords(c)].second.first - b.b)/1.0/s->ep[convertCoords(c)].second.first+(s->ep[convertCoords(c)].second.second - b.h)/1.0/s->ep[convertCoords(c)].second.second;
    relativeDifference*=1000000;
//    float relativeDifference =(s->ep[convertCoords(c)].first - b.l)+(s->ep[convertCoords(c)].second.first - b.b)+(s->ep[convertCoords(c)].second.second - b.h);
    r+=relativeDifference;
    return r;
}
struct Dimensions
{
    int length, width, height;
};

struct Position
{
    int x, y, z;
};

enum PacketType
{
    Priority,
    Economy
};

struct ULD
{
    Dimensions dimensions;
    int id, weight;
    ULD(Dimensions dim, int i, int wt)
        : dimensions(dim), id(i), weight(wt) {}
};
Uld convertULDToUld(ULD x){
    Uld r;
    r.dim.l = x.dimensions.length;
    r.dim.b = x.dimensions.width;
    r.dim.h = x.dimensions.height;
    r.weight = 0;
    r.maxWt = x.weight;
    r.com.x = r.dim.l/2;
    r.com.x = r.dim.l/2;
    r.com.x = r.dim.l/2;
    r.ID = x.id;
    r.maxBound.x = r.maxBound.y = r.maxBound.z=0;
    return r;
}
struct Packet
{
    Dimensions dimensions;
    int id, weight, uld_num;
    PacketType type;
    optional<int> cost;
    Packet(Dimensions dim, int i, int wt, int num, PacketType t, optional<int> val = nullopt)
        : dimensions(dim), id(i), weight(wt), uld_num(num), type(t), cost(val) {}
};
Box convertPacketToBox(Packet x){
    Box r;
    r.weight = x.weight;
    r.ID = x.id;
    r.cost = x.cost?x.cost.value():0;
    r.isPriority = x.type == Priority;
    r.l = x.dimensions.length;
    r.b = x.dimensions.width;
    r.h = x.dimensions.height;
    return r;
}

struct Encoding
{
    vector<double> sequence, orientation;
    int fitness;

    Encoding() : sequence(), orientation(), fitness(0) {}

    Encoding(const vector<double> &seq, const vector<double> &orient, int fit)
        : sequence(seq), orientation(orient), fitness(fit) {}
};
class Genetic
{
private:
    vector<ULD> ulds;
    vector<Packet> packets;
    int num_packets, num_generations, population_size;
    mt19937 random_gen;

    set<int> RandomIndices(int n, int k)
    {
        uniform_int_distribution<> dis(0, n - 1);
        set<int> random_indices;

        while (random_indices.size() < k)
        {
            random_indices.insert(dis(random_gen));
        }

        return random_indices;
    }

    int FitnessFunction(Encoding solution)
    {
        for (int i = 0; i < num_packets; i++)
        {
            solution.orientation[i] = static_cast<int>(floor(solution.orientation[i] * 6));
        }
        vector<int> permutation(num_packets);

        iota(permutation.begin(), permutation.end(), 1);

        sort(permutation.begin(), permutation.end(), [&solution](size_t a, size_t b)
             { return solution.sequence[a - 1] < solution.sequence[b - 1]; });

        /*
        TODO: using the permutation and orientation, pack all the packets and evaluate the fitness using packing effiency and cost
        */
        Sorter t;t.val = [](Box a,Box b){return true;};
        vector<Box>b;
        for(int i:permutation){
            b.emplace_back(convertPacketToBox(packets[i-1]));
        }
        vector<Uld>u;
        For(i,ulds.size())u.emplace_back(convertULDToUld(ulds[i]));
        Merit Residue;Residue.val = residueFunc;
        Solver s(t, Residue, b, u);
        s.solve();
        return s.cost();
    }

    vector<Encoding> Warmstart(int factor = WARMSTART_FACTOR)
    {
        cout << "Generation 1 started." << endl;

        uniform_real_distribution<> dist_priority(0.0, 0.5), dist_economy(0.5, 1.0);

        vector<Encoding> population(factor * population_size);
        for (int i = 0; i < population.size(); i++)
        {
            population[i].sequence.resize(num_packets), population[i].orientation.resize(num_packets);
            for (int j = 0; j < num_packets; j++)
            {
                if (packets[j].type == Priority)
                {
                    population[i].sequence[j] = dist_priority(random_gen);
                    population[i].orientation[j] = dist_priority(random_gen);
                }
                else
                {
                    population[i].sequence[j] = dist_economy(random_gen);
                    population[i].orientation[j] = dist_economy(random_gen);
                }
            }
            population[i].fitness = FitnessFunction(population[i]);
        }
        vector<int> indices(factor * population_size);

        iota(indices.begin(), indices.end(), 0);

        sort(indices.begin(), indices.end(), [&population](size_t a, size_t b)
             { return population[a].fitness > population[b].fitness; });

        vector<Encoding> initial_population(population_size);
        for (int i = 0; i < population_size; i++)
        {
            initial_population[i] = population[indices[i]];
        }

        return initial_population;
    }

    pair<Encoding, Encoding> SelectParents(const vector<Encoding> &current_population, int tournament_factor = TOURNAMENT_FACTOR)
    {
        set indices = RandomIndices(population_size, tournament_factor);

        vector<pair<int, int>> fitness_index_pair;

        for (int index : indices)
        {
            fitness_index_pair.push_back({current_population[index].fitness, index});
        }

        sort(fitness_index_pair.begin(), fitness_index_pair.end(), greater<pair<int, int>>());

        return {current_population[fitness_index_pair[0].second], current_population[fitness_index_pair[1].second]};
    }

    vector<Encoding> Crossover(const vector<Encoding> &current_population, int num_offsprings)
    {
        vector<Encoding> offsprings;
        for (int i = 1; i <= num_offsprings; i++)
        {
            pair<Encoding, Encoding> parents = SelectParents(current_population);
            set<int> indices = RandomIndices(num_packets, 2);
            int start = *indices.begin(), end = *indices.rbegin();
            vector<double> seq(num_packets), orient(num_packets);
            for (int j = 0; j < num_packets; j++)
            {
                if ((j < start) || (j > end))
                {
                    seq[j] = parents.second.sequence[j];
                    orient[j] = parents.second.orientation[j];
                }
                else
                {
                    seq[j] = parents.first.sequence[j];
                    orient[j] = parents.first.orientation[j];
                }
            }
            int fitness = FitnessFunction(Encoding(seq, orient, INT_MIN));
            offsprings.push_back(Encoding(seq, orient, fitness));
        }
        return offsprings;
    }

    Encoding GenerateNeighbor(const Encoding &solution, int neighborhood_range = NEIGHBORHOOD_RANGE)
    {
        Encoding neighbor = solution;

        uniform_int_distribution<int> index_dist(0, num_packets - 1);
        uniform_real_distribution<double> change_dist(-0.1, 0.1);

        int i = index_dist(random_gen);

        int lower_bound = max(0, i - neighborhood_range);
        int upper_bound = min(num_packets - 1, i + neighborhood_range);
        uniform_int_distribution<int> neighbor_dist(lower_bound, upper_bound);

        int j = neighbor_dist(random_gen);

        swap(neighbor.sequence[i], neighbor.sequence[j]);

        int k = index_dist(random_gen);
        neighbor.orientation[k] = max(0.0, min(1.0, neighbor.orientation[k] + change_dist(random_gen)));

        neighbor.fitness = FitnessFunction(neighbor);

        return neighbor;
    }

    void TabuSearch(Encoding &solution, int max_iter = MAX_TABU_ITERS, int tabu_size = TABU_SIZE, int neighborhood_size = NEIGHBORHOOD_SIZE)
    {
        deque<Encoding> tabu_list;

        Encoding best_solution = solution;

        for (int iter = 0; iter < max_iter; ++iter)
        {
            vector<Encoding> neighbors;

            for (int i = 0; i < neighborhood_size; ++i)
            {
                neighbors.push_back(GenerateNeighbor(solution));
            }

            Encoding best_neighbor = solution;
            int best_neighbor_fitness = INT_MIN;
            for (const auto &neighbor : neighbors)
            {
                bool is_tabu = false;
                for (const auto &tabu_solution : tabu_list)
                {
                    if (neighbor.sequence == tabu_solution.sequence && neighbor.orientation == tabu_solution.orientation)
                    {
                        is_tabu = true;
                        break;
                    }
                }

                if (!is_tabu || neighbor.fitness > best_solution.fitness)
                {
                    if (neighbor.fitness > best_neighbor_fitness)
                    {
                        best_neighbor = neighbor;
                        best_neighbor_fitness = neighbor.fitness;
                    }
                }
            }

            tabu_list.push_back(best_neighbor);
            if (tabu_list.size() > tabu_size)
            {
                tabu_list.pop_front();
            }

            if (best_neighbor_fitness > best_solution.fitness)
            {
                best_solution = best_neighbor;
            }
        }

        solution = best_solution;
    }

    void Mutation(vector<Encoding> &current_population, int mutation_prob = MUTATION_PROB)
    {
        uniform_real_distribution<> dist(0.0, 1.0);

        for (int i = 0; i < current_population.size(); i++)
        {
            if (dist(random_gen) < mutation_prob)
            {
                TabuSearch(current_population[i]);
            }
        }
    }

public:
    Genetic(const vector<ULD> &ulds, const vector<Packet> &packets, int n = NUM_PACKETS, int num_gen = NUM_GENERATIONS, int pop_size = POPULATION_SIZE)
        : ulds(ulds), packets(packets), num_packets(n), num_generations(num_gen), population_size(pop_size)
    {
        random_device rd;
        random_gen = mt19937(rd());
    }

    void Execute()
    {
        cout << "Execution started. Total generations: " << num_generations << endl;

        ofstream top_solutions_file("top_solutions.txt");

        vector<Encoding> current_population = Warmstart();

        for (int i = 2; i <= num_generations; i++)
        {
            cout << "Generation " << i << " started." << endl;

            vector<Encoding> new_generation(population_size);
            int num_elites = population_size * ELITISM_PERCENTAGE;
            for (int j = 0; j < num_elites; j++)
            {
                new_generation[j] = current_population[j];
            }
            vector<Encoding> offsprings = Crossover(current_population, population_size - num_elites);
            for (int j = num_elites; j < population_size; j++)
            {
                new_generation[j] = offsprings[j - num_elites];
            }
            Mutation(new_generation);

            sort(new_generation.begin(), new_generation.end(), [](Encoding a, Encoding b)
                 { return a.fitness > b.fitness; });

            top_solutions_file << "Generation " << i << ":\n";
            for (int j = 0; j < min(BEST_K_SOLNS, (int)new_generation.size()); j++)
            {
                Encoding &solution = new_generation[j];

                vector<int> transformed_orientation(num_packets);
                for (int k = 0; k < num_packets; k++)
                {
                    transformed_orientation[k] = static_cast<int>(floor(solution.orientation[k] * 6));
                }

                vector<int> permutation(num_packets);
                iota(permutation.begin(), permutation.end(), 0);
                sort(permutation.begin(), permutation.end(), [&solution](size_t a, size_t b)
                    { return solution.sequence[a] < solution.sequence[b]; });

                top_solutions_file << "  Solution " << j + 1 << " Fitness: " << solution.fitness << "\n";
                top_solutions_file << "    Sequence: ";
                for (int k = 0; k < num_packets; k++)
                {
                    top_solutions_file << permutation[k] << " ";
                }

                top_solutions_file << "\n";
                top_solutions_file << "    Orientation: ";
                for (int k = 0; k < num_packets; k++)
                {
                    top_solutions_file << transformed_orientation[k] << " ";
                }
                top_solutions_file << "\n";
            }
            top_solutions_file << "\n";

            current_population = new_generation;

            cout << "Best Fitness of Generation " << i << ": " << new_generation[0].fitness << endl;

            cout << "Generation " << i << " completed." << endl;
        }

        top_solutions_file.close();

        cout << "Execution completed." << endl;
    }
};

PacketType ParsePacketType(const string &type)
{
    return (type == "Priority") ? Priority : Economy;
}

optional<int> ParseCost(const string &cost)
{
    return (cost == "-") ? nullopt : optional<int>(stoi(cost));
}

void ParseULDs(const string &filename, vector<ULD> &ulds)
{
    ifstream infile(filename);
    if (!infile)
    {
        cerr << "Error opening ULD file: " << filename << endl;
        return;
    }

    string line;
    int uld_id = 1;
    while (getline(infile, line))
    {
        stringstream line_stream(line);
        string identifier, length, width, height, weight;
        getline(line_stream, identifier, ',');
        getline(line_stream, length, ',');
        getline(line_stream, width, ',');
        getline(line_stream, height, ',');
        getline(line_stream, weight, ',');

        Dimensions dim = {stoi(length), stoi(width), stoi(height)};
        ulds.emplace_back(dim, uld_id++, stoi(weight));
    }

    infile.close();
}

void ParsePackets(const string &filename, vector<Packet> &packets)
{
    ifstream infile(filename);
    if (!infile)
    {
        cerr << "Error opening Packet file: " << filename << endl;
        return;
    }

    string line;
    int package_id = 1;
    while (getline(infile, line))
    {
        stringstream line_stream(line);
        string identifier, length, width, height, weight, type, cost;
        getline(line_stream, identifier, ',');
        getline(line_stream, length, ',');
        getline(line_stream, width, ',');
        getline(line_stream, height, ',');
        getline(line_stream, weight, ',');
        getline(line_stream, type, ',');
        getline(line_stream, cost, ',');
        Dimensions dim = {stoi(length), stoi(width), stoi(height)};
        PacketType packet_type = ParsePacketType(type);
        optional<int> delay_cost = ParseCost(cost);
        packets.emplace_back(dim, package_id++, stoi(weight), -1, packet_type, delay_cost);
    }
    infile.close();
}

//int main()
//{
//    vector<ULD> ulds;
//    vector<Packet> packets;
//
//    string uld_file = "uld.txt";
//    string packet_file = "packets.txt";
//
//    ParseULDs(uld_file, ulds);
//    ParsePackets(packet_file, packets);
//
//    Genetic genetic(ulds, packets);
//    genetic.Execute();
//
//    return 0;
//}

//--------------------------------------------------------------------
using namespace std;
vector<Uld> ULDList(6);
vector<Box>dat(400);
const int LevelXYBoundWeight  =10;
void f(int __) {
    Sorter Vol_Ht;
    Vol_Ht.val = [](Box a,Box b){
        if(b.isPriority and (not a.isPriority))return false;
        if(a.isPriority and (not b.isPriority))return true;
        if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))>min(b.h,min(b.b,b.l));
        return a.l*a.b*a.h > b.l*b.b*b.h;
    };
    Sorter Ht_Vol;
    Ht_Vol.val = [](Box a,Box b){
        if(a.h==b.h)return a.l*a.b*a.h > b.l*b.b*b.h;
        return a.h>b.h;
    };
    Sorter Area_Ht;
    Area_Ht.val = [](Box a,Box b){
        if(a.l*a.b==b.l*b.b)return a.h>b.h;
        return a.l*a.b > b.l*b.b;
    };
    
    //merits
    Merit MinVol;//redundant
    MinVol.val = [](coords c, Box b,Solver* s){
        return 1;
    };
    Merit minXYBound;
    minXYBound.val = [](coords c, Box box,Solver* s){
        int ret = 0;
        int b = c.box;
        if(c.x+box.l>(s->ULDl[b].maxBound.x))ret+=(s->ULDl[b].maxBound.x)-(c.x+box.l);
        if(c.y+box.b>(s->ULDl[b].maxBound.y))ret+=(s->ULDl[b].maxBound.y)-(c.y+box.b);
        return ret;
    };
    Merit levelXYBound;
    levelXYBound.val = [](coords c, Box box,Solver* s){
        int ret = 0;
        int b = c.box;
        if(c.x+box.l>(s->ULDl[b].maxBound.x))ret+=((s->ULDl[b].maxBound.x)-(c.x+box.l))*LevelXYBoundWeight;
        else ret+=-(s->ULDl[b].maxBound.x)+(c.x+box.l);
        if(c.y+box.b>(s->ULDl[b].maxBound.y))ret+=((s->ULDl[b].maxBound.y)-(c.y+box.b))*LevelXYBoundWeight;
        else ret+=-(s->ULDl[b].maxBound.y)+(c.y+box.b);
        return ret;
    };
    Merit Residue;
    Residue.val = residueFunc;
    
    ULDList[0].dim.l = 224; ULDList[0].dim.b= 318; ULDList[0].dim.h = 162;ULDList[0].weight = 100;ULDList[0].maxBound.x =ULDList[0].maxBound.y=ULDList[0].maxBound.z = 0;
    freopen("/Users/agupta/Desktop/q/cpp/cpp/ULD.in", "r" , stdin);
    For(i,ULDList.size()){
        ULDList[i].weight=0;
        cin>>ULDList[i].dim.l>>ULDList[i].dim.b>>ULDList[i].dim.h>>ULDList[i].maxWt;ULDList[i].maxBound.x =ULDList[i].maxBound.y=ULDList[i].maxBound.z = 0;
        ULDList[i].com.x = ULDList[i].dim.l/2;ULDList[i].com.y = ULDList[i].dim.b/2;ULDList[i].com.z = ULDList[i].dim.h/2;
    }
//    freopen("/Users/agupta/Desktop/q/cpp/cpp/outpus.out", " gw" , stdout);
    freopen("/Users/agupta/Desktop/q/cpp/cpp/package.in", "r" , stdin);
//    For(i,2)dat[i].l =dat[i].b =dat[i].h = i+1;
//    dat[0].l = 99; dat[0].b= 53; dat[0].h = 55;
//    dat[1].l = 56; dat[1].b= 99; dat[1].h = 81;
//    dat[2].l = 42; dat[2].b= 101; dat[2].h = 51;
//    dat[3].l = 108; dat[3].b= 75; dat[3].h = 56;
//    dat[4].l = 88; dat[4].b= 58; dat[4].h = 64;
//    dat[5].l = 91; dat[5].b= 56; dat[5].h = 84;
//    dat[6].l = 88; dat[6].b= 78; dat[6].h = 93;
//    dat[7].l = 108; dat[7].b= 105; dat[7].h = 76;
//    dat[8].l = 73; dat[8].b= 71; dat[8].h = 88;
//    dat[9].l = 88; dat[9].b= 70; dat[9].h = 85;
//    dat[10].l = 55; dat[10].b= 80; dat[10].h = 81;
//    dat[11].l = 48; dat[11].b= 80; dat[11].h = 88;
//    dat[12].l = 55; dat[12].b= 94; dat[12].h = 87;
    For(i,dat.size()){
        char c;cin>>c;cin>>c;
        cin>>dat[i].ID>>dat[i].l>>dat[i].b>>dat[i].h>>dat[i].weight;
        string s;
        cin>>s;
        dat[i].isPriority = s=="Priority";
        cin>>dat[i].cost;
    }
    Solver s(Vol_Ht, Residue, dat, ULDList);
    s.solve();
    For(i,dat.size()){
        cout<<s.data[i].ID<<","<<s.placement[i].first.box+1<<","<<s.placement[i].first.x<<","<<s.placement[i].first.y<<","<<s.placement[i].first.z<<","<<s.placement[i].second.l+s.placement[i].first.x<<","<<s.placement[i].second.b+s.placement[i].first.y<<","<<s.placement[i].second.h+s.placement[i].first.z<<"\n";
    }
    int usedVol = 0, ULDVol =0;
    For(i,s.ULDl.size()){
        int vol =0;
        for(int b:s.ULDPackages[i])vol+=s.data[b].l*s.data[b].b*s.data[b].h;
//        cout<<1.0*vol/s.ULDl[i].dim.l/s.ULDl[i].dim.b/s.ULDl[i].dim.h<<"\n";
        usedVol+=vol;
        ULDVol+=s.ULDl[i].dim.l*s.ULDl[i].dim.b*s.ULDl[i].dim.h;
    }
    cout<<1.0*usedVol/ULDVol<<"\n";
    cout<<s.cost()<<"\n";
    
//    vector<Packet>pc;ParsePackets("/Users/agupta/Desktop/q/cpp/cpp/packageNormal.txt",pc);
//    vector<struct ULD>u;ParseULDs("/Users/agupta/Desktop/q/cpp/cpp/ULDNormal.txt", u);
//    Genetic gen(u, pc);//    gen.Execute();
//    gen.Execute();
}

signed main(){
    ios_base::sync_with_stdio(0); cin.tie(0);cout.tie();
    int t=1;
//    cin>>t;
    for(int i = 0; i<t;i++){
//        #ifdef META
//        cout<<"Case #"<<i+1<<": ";
//        #endif
        f(1);
    }
}
