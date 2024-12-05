#include "solver.h"

using namespace std;

extern double weightz, power_fac;

/*
*  @brief: check if two packages collide
*  @param: e: coordinates of the first package
*/

bool check(const pair<pair<int,int>,pair<int,int>>&a, const pair<pair<int,int>,pair<int,int>>&b)
        {
            int p=a.second.second;
            int q=b.second.second;
            // return true;
            return (p>q);
        }
Solver::Solver(Sorter sorter_, Merit merit_, vector<Box> boxes, vector<Uld> ULD_)
{
    this->merit = merit_;
    this->sorter = sorter_;
    //        this->merit.init(this);
    //        this->sorter.init(this);
    this->data = boxes;
    solveTill = boxes.size();
    def.x = def.y = def.z = def.box = -1;
    Box def_;
    def_.l = def_.b = def_.h = -1;
    this->placement.assign(boxes.size(), pair<coords, Box>(def, def_));
    ULDl = ULD_;
    ULDPackages.assign(ULDl.size(), set<int>());
    surfaces.assign(ULDl.size(), set<pair<int,pair<pair<int,int>,pair<int,int>>>>());
    ULDHasPriority.assign(ULDl.size(), false);

    // comparator function definitoins for additional EP creation
    compare_x = [this](const int &a, const int &b) { return placement[a].first.x + placement[a].second.l > placement[b].first.x + placement[b].second.l; };
    compare_y = [this](const int &a, const int &b) { return placement[a].first.y + placement[a].second.b > placement[b].first.y + placement[b].second.b; };
    compare_z = [this](const int &a, const int &b) { return placement[a].first.z + placement[a].second.h > placement[b].first.z + placement[b].second.h; };

    ULD_sorted_x.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_x));
    ULD_sorted_y.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_y));
    ULD_sorted_z.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_z));

    // sort blocking pkts in increasing order of base face coordinate
    compare_x_base = [this](const int &a, const int &b) { return placement[a].first.x < placement[b].first.x; }; 
    compare_y_base = [this](const int &a, const int &b) { return placement[a].first.y < placement[b].first.y; };
    compare_z_base = [this](const int &a, const int &b) { return placement[a].first.z < placement[b].first.z; };

    ULD_blocking_boxes_x = set<int, function<bool(const int &, const int &)>>(compare_x_base);
    ULD_blocking_boxes_y = set<int, function<bool(const int &, const int &)>>(compare_y_base);
    ULD_blocking_boxes_z = set<int, function<bool(const int &, const int &)>>(compare_z_base);
}

bool Solver::writeToFile(string filename){
    set<int> ULDPackages;
    int pkgCount = 0;
    for (int i = 0; i < data.size(); ++i) {
        if (placement[i].first.x != -1) {
            if (data[i].isPriority)
                ULDPackages.insert(placement[i].first.box);
            pkgCount++;
        }
    }
    ofstream file;
    file.open(filename);
    if(!file.is_open())return false;
    file << -this->cost() << "," << pkgCount << "," << ULDPackages.size() << "\n";
    for(int i = 0; i < data.size(); ++i){
        if(placement[i].first.x == -1){
            file << "P-" << data[i].ID << ",NONE,-1,-1,-1,-1,-1,-1\n";
        } else {
            file << "P-" << data[i].ID << "," << "U" << placement[i].first.box + 1 << "," 
                << placement[i].first.x << "," << placement[i].first.y << "," 
                << placement[i].first.z << "," 
                << placement[i].second.l + placement[i].first.x << "," 
                << placement[i].second.b + placement[i].first.y << "," 
                << placement[i].second.h + placement[i].first.z << "\n";
        }
    }
    file << "Ordering: {";
    for(auto it: data){
        file << it.ID << ", ";
    } 
    file << "}\n";
    file.close();
    return true;
}

int Solver::cost()
{
    int c = 0;
    int count=0;
    set<int> priorityShipments;
    For(i, data.size())
    {
        if (placement[i].first.x == -1)
        {
            c -= data[i].cost;
            if (data[i].isPriority)
            {
                count++;
                c -= NON_PRIORITY_COST;
            }
        }
        else
        {
            //                c+=data[i].cost;

            if (data[i].isPriority)
                priorityShipments.insert(placement[i].first.box);
        }
    }
    c -= priorityShipments.size() * PRIORITY_ULD_COST;
    //        For(i,ULDl.size())c-=abs(ULDl[i].dim.l/2-ULDl[i].com.x)+abs(ULDl[i].dim.b/2-ULDl[i].com.y)+abs(ULDl[i].dim.h/2-ULDl[i].com.z);
    //        cout<<c<<"\n";
    // cout << "Priority left" << count << "\n";
    // cout << priorityShipments.size() << "\n";
    return c;
}
bool Solver::checkCollision(coords e, Box b)
{
    // checks collision of prespective packages with all other packages of sam eULD
    // if (e.x + b.l >= ULDl[e.box].dim.l or e.y + b.b >= ULDl[e.box].dim.b or e.z + b.h >= ULDl[e.box].dim.h or ULDl[e.box].weight + b.weight > ULDl[e.box].maxWt)
    if (e.x + b.l > ULDl[e.box].dim.l or e.y + b.b > ULDl[e.box].dim.b or e.z + b.h > ULDl[e.box].dim.h or ULDl[e.box].weight + b.weight > ULDl[e.box].maxWt)
        return true;
    for (auto i : ULDPackages[e.box])
    {
        auto x = placement[i];
        if ((x.first.x < b.l + e.x and x.first.y < b.b + e.y and x.first.z < b.h + e.z and e.x < x.first.x + x.second.l and e.y < x.first.y + x.second.b and e.z < x.first.z + x.second.h))
        {
            return true;
        }
    }
    return false;
}

void Solver::setSolveFrom(int _solveFrom)
{
    solveFrom = _solveFrom;
}
void Solver::setSolveTill(int _solveTill)
{
    solveTill = _solveTill;
}
void Solver::resetSolveTill()
{
    solveTill = data.size();
}
void Solver::solve()
{
    // int c=0;
    // cout << weightz << endl;
    #ifndef GENETIC
    #ifdef OLD_SORT
    sort(data.begin(), data.end(), this->sorter.val);
    #endif
    #ifndef OLD_SORT

    
    #endif
    #endif
    
    

    if(solveFrom == 0){
        this->sorter.val(data);
        for(int i=0; i<ULDl.size();i++) {
            ep[pair<int, pair<int, pii>>(i, pair<int, pii>(0, pii(0, 0)))] = pair<int, pii>(ULDl[i].dim.l, pii(ULDl[i].dim.b, ULDl[i].dim.h));
        }
    }
    
    // for (auto &box : data) {
    //     cout << box.ID << " ";
    // }
    for(int i = solveFrom; i < solveTill; i++)
    {
        // cout << ep.size() << " ";
        Box b = data[i];
        pair<int, pair<coords, Box>> best;
        best.first = -INF;
        vector<Box> perms(6);
        perms[0].l = b.l;
        perms[0].b = b.b;
        perms[0].h = b.h; 
        perms[1].l = b.l;
        perms[1].h = b.b;
        perms[1].b = b.h;
        perms[2].l = b.b;
        perms[2].b = b.l;
        perms[2].h = b.h;
        perms[3].l = b.b;
        perms[3].b = b.h;
        perms[3].h = b.l;
        perms[4].l = b.h;
        perms[4].b = b.b;
        perms[4].h = b.l;
        perms[5].l = b.h;
        perms[5].b = b.l;
        perms[5].h = b.b;
        for (Box p : perms)
            for (auto x : ep)
            {
                // check if it can fit
                coords e;
                e.box = x.first.first;
                e.x = x.first.second.first;
                e.y = x.first.second.second.first;
                e.z = x.first.second.second.second;
                if(checkCollision(e,p))
                    continue;
                p.isPriority = b.isPriority;
                int score = this->merit.val(e, p, this);
                if (best.first < score)
                    best = pair<int, pair<coords, Box>>(score, pair<coords, Box>(e, p));
            }
        // update vals
        // cout << best.first << " " << best.second.first.x << " " << best.second.first.y << " " << best.second.first.z << " " << best.second.second.l << " " << best.second.second.b << " " << best.second.second.h << endl;
        if (best.first == -INF)
            continue;
        if (data[i].isPriority)
            ULDHasPriority[best.second.first.box] = true;
        placement[i] = best.second;
        ULDPackages[best.second.first.box].insert(i);
        ULD_sorted_x[best.second.first.box].insert(i);
        ULD_sorted_y[best.second.first.box].insert(i);
        ULD_sorted_z[best.second.first.box].insert(i);
        // int pp=placement[i].first.z;
        gravity_pull(i);
        // if(pp!=placement[i].first.z)
        // c++;
        // addEP(i);
        addEP2(i);
        update(i);
        // if(placement[i].second==data[i])
        // if(checkGravity(placement[i].first, placement[i].second))
        // printf("error\n");
    }
    cout << "EP size: " << ep.size() << endl;
    // cout << c;

}

void Solver::update(int i)
{
    int b = placement[i].first.box;
    updateMaxBound(i);
    updateResidue(i);
    ULDl[b].com.x += (placement[i].first.x + placement[i].second.l / 2) * data[i].weight;
    ULDl[b].com.y += (placement[i].first.y + placement[i].second.b / 2) * data[i].weight;
    ULDl[b].com.z += (placement[i].first.z + placement[i].second.h / 2) * data[i].weight;
    ULDl[b].weight += data[i].weight;
    surfaces[b].insert(make_pair(placement[i].first.z+placement[i].second.h,make_pair(make_pair(placement[i].first.x,placement[i].first.y),make_pair(placement[i].first.x + placement[i].second.l,placement[i].first.y + placement[i].second.b))));
}


vector<coords> Solver::getCOM()
{
    vector<coords> r(ULDl.size());
    For(i, r.size())
    {
        if (ULDl[i].weight == 0)
        {
            r[i].x = ULDl[i].dim.l / 2;
            r[i].y = ULDl[i].dim.b / 2;
            r[i].z = ULDl[i].dim.h / 2;
            continue;
        }
        r[i].x = ULDl[i].com.x / ULDl[i].weight;
        r[i].y = ULDl[i].com.y / ULDl[i].weight;
        r[i].z = ULDl[i].com.z / ULDl[i].weight;
    }
    return r;
}

pair<int, pii> Solver::getResidueSpace(coords src)
{
    pair<int, pii> ret(beamprojectXPos(src).x - src.x, pii(beamprojectYPos(src).y - src.y, beamprojectZPos(src).z - src.z));
    return ret;
}
void Solver::initialise(coords &X, coords &ob, int x, int y, int z)
{
    X.x = ob.x + x;
    X.y = ob.y + y;
    X.z = ob.z + z;
    X.box = ob.box;
}

coords Solver::beamprojectZNeg(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 1, 0, 0);
    initialise(q, ob1, 0, 1, 0);
    initialise(r, ob1, 1, 1, 0);
    coords p1 = rayProjectZNeg(p);
    coords q1 = rayProjectZNeg(ob1);
    coords r1 = rayProjectZNeg(q);
    coords s1 = rayProjectZNeg(r);
    coords ans;
    ans.x = ob1.x;
    ans.y = ob1.y;
    ans.z = min(min(min(p1.z, q1.z), r1.z), s1.z);
    ans.box = ob1.box;
    return ans;
}

coords Solver::beamprojectYNeg(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 1, 0, 0);
    initialise(q, ob1, 0, 0, 1);
    initialise(r, ob1, 1, 0, 1);
    coords p1 = rayProjectYNeg(p);
    coords q1 = rayProjectYNeg(ob1);
    coords r1 = rayProjectYNeg(q);
    coords s1 = rayProjectYNeg(r);
    coords ans;
    ans.x = ob1.x;
    ans.y = min(min(min(p1.y, q1.y), r1.y), s1.y);
    ans.z = ob1.z;
    ans.box = ob1.box;
    return ans;
}
coords Solver::beamprojectXNeg(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 0, 1, 0);
    initialise(q, ob1, 0, 0, 1);
    initialise(r, ob1, 0, 1, 1);
    coords p1 = rayProjectXNeg(p);
    coords q1 = rayProjectXNeg(ob1);
    coords r1 = rayProjectXNeg(q);
    coords s1 = rayProjectXNeg(r);
    coords ans;
    ans.x = min(min(min(p1.x, q1.x), r1.x), s1.x);
    ans.y = ob1.y;
    ans.z = ob1.z;
    ans.box = ob1.box;
    return ans;
}
coords Solver::beamprojectZPos(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 1, 0, 0);
    initialise(q, ob1, 0, 1, 0);
    initialise(r, ob1, 1, 1, 0);
    coords p1 = rayProjectZPos(p);
    coords q1 = rayProjectZPos(ob1);
    coords r1 = rayProjectZPos(q);
    coords s1 = rayProjectZPos(r);
    coords ans;
    ans.x = ob1.x;
    ans.y = ob1.y;
    ans.z = min(min(min(p1.z, q1.z), r1.z), s1.z);
    ans.box = ob1.box;
    return ans;
}
coords Solver::beamprojectYPos(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 1, 0, 0);
    initialise(q, ob1, 0, 0, 1);
    initialise(r, ob1, 1, 0, 1);
    coords p1 = rayProjectYPos(p);
    coords q1 = rayProjectYPos(ob1);
    coords r1 = rayProjectYPos(q);
    coords s1 = rayProjectYPos(r);
    coords ans;
    ans.x = ob1.x;
    ans.y = min(min(min(p1.y, q1.y), r1.y), s1.y);
    ans.z = ob1.z;
    ans.box = ob1.box;
    return ans;
}
coords Solver::beamprojectXPos(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 0, 1, 0);
    initialise(q, ob1, 0, 0, 1);
    initialise(r, ob1, 0, 1, 1);
    coords p1 = rayProjectXPos(p);
    coords q1 = rayProjectXPos(ob1);
    coords r1 = rayProjectXPos(q);
    coords s1 = rayProjectXPos(r);
    coords ans;
    ans.x = min(min(min(p1.x, q1.x), r1.x), s1.x);
    ans.y = ob1.y;
    ans.z = ob1.z;
    ans.box = ob1.box;
    return ans;
}
bool Solver::check_collision_2D(int x1_min, int x1_max, int y1_min, int y1_max, int x2_min, int x2_max, int y2_min, int y2_max)
{
    bool is_x2_min_inside = (x2_min >= x1_min) && (x2_min <= x1_max);
    bool is_x2_max_inside = (x2_max > x1_min) && (x2_max <= x1_max);
    bool is_y2_min_inside = (y2_min >= y1_min) && (y2_min <= y1_max);
    bool is_y2_max_inside = (y2_max > y1_min) && (y2_max <= y1_max);
    return (is_x2_min_inside || is_x2_max_inside) && (is_y2_min_inside || is_y2_max_inside);
}
void Solver::projection_neg_z_advanced(vector<coords> &advanced_eps, coords start, int self_pid){

    bool hit_bottom = true;
    for(auto i: ULD_sorted_z[start.box])
    {
        coords potential_ep;
        potential_ep.box = start.box;
        potential_ep.x = start.x;
        potential_ep.y = start.y;
        potential_ep.z = placement[i].first.z + placement[i].second.h;

        if(i == self_pid)
            continue;
        auto pkt = placement[i];
        if(pkt.first.x == -1) continue;
        if(pkt.first.x + pkt.second.l <= start.x) continue;
        if(pkt.first.y + pkt.second.b <= start.y) continue;
        if(pkt.first.z >= start.z) continue;

        // case 0: ray hits the object (no more projections)
        if((start.x >= pkt.first.x) && (start.x < pkt.first.x + pkt.second.l) &&
           (start.y >= pkt.first.y) && (start.y < pkt.first.y + pkt.second.b)) {
            advanced_eps.push_back(potential_ep);
            hit_bottom = false;
            break;
        }
        // check if the pkt is being blocked by other packets
        if(start.x >= pkt.first.x){
            bool is_blocked = false;
            for(auto j: ULD_blocking_boxes_z){
                auto blocking_pkt = placement[j];
                if(blocking_pkt.first.x == -1) continue; // although this should not happen
                if(blocking_pkt.first.z > pkt.first.z + pkt.second.h) continue;
                if(!((blocking_pkt.first.y + blocking_pkt.second.b > start.y) && (blocking_pkt.first.y <= pkt.first.y))) continue;
                if(!((start.x >= blocking_pkt.first.x) && (start.x < blocking_pkt.first.x + blocking_pkt.second.l))) continue;
                is_blocked = true;
                break;
            }
            if(is_blocked) continue;
        }
        else if(start.y >= pkt.first.y){
            // give line by line 
            bool is_blocked = false;
            for(auto j: ULD_blocking_boxes_z){
                auto blocking_pkt = placement[j];
                if(blocking_pkt.first.x == -1) continue; // although this should not happen
                if(blocking_pkt.first.z > pkt.first.z + pkt.second.h) continue;
                if(!((blocking_pkt.first.x + blocking_pkt.second.l > start.x) && (blocking_pkt.first.x <= pkt.first.x))) continue;
                if(!((start.y >= blocking_pkt.first.y) && (start.y < blocking_pkt.first.y + blocking_pkt.second.b))) continue;
                is_blocked = true;
                break;
            }
            if(is_blocked) continue;   
        }
        else{ // diagonal intersection
            bool is_blocked = false;
            for(auto j: ULD_blocking_boxes_z){
                auto blocking_pkt = placement[j];
                if(blocking_pkt.first.x == -1) continue; // although this should not happen
                if(blocking_pkt.first.z > pkt.first.z + pkt.second.h) continue;
                is_blocked = check_collision_2D(pkt.first.x, pkt.first.x + pkt.second.l, 
                                                pkt.first.y, pkt.first.y + pkt.second.b,
                                                blocking_pkt.first.x, blocking_pkt.first.x + blocking_pkt.second.l, 
                                                blocking_pkt.first.y, blocking_pkt.first.y + blocking_pkt.second.b);
                if(is_blocked) break;
            }
            if(is_blocked) continue;   
        }
        
        // add to the blocking pkts list
        ULD_blocking_boxes_z.insert(i);
        if(pkt.first.z + pkt.second.h < start.z){
            advanced_eps.push_back(potential_ep);
        }
    }
    ULD_blocking_boxes_z.clear();
    if(hit_bottom){
        coords potential_ep;
        potential_ep.box = start.box;
        potential_ep.x = start.x;
        potential_ep.y = start.y;
        potential_ep.z = 0;
        advanced_eps.push_back(potential_ep);
    }
    return;

}

void Solver::projection_neg_y_advanced(vector<coords> &advanced_eps, coords start, int self_pid){
    
        bool hit_bottom = true;
        for(auto i: ULD_sorted_y[start.box])
        {
            coords potential_ep;
            potential_ep.box = start.box;
            potential_ep.x = start.x;
            potential_ep.y = placement[i].first.y + placement[i].second.b;
            potential_ep.z = start.z;
    
            if(i == self_pid)
                continue;
            auto pkt = placement[i];
            if(pkt.first.x == -1) continue;
            if(pkt.first.x + pkt.second.l <= start.x) continue;
            if(pkt.first.z + pkt.second.h <= start.z) continue;
            if(pkt.first.y >= start.y) continue;
    
            // case 0: ray hits the object (no more projections)
            if((start.x >= pkt.first.x) && (start.x < pkt.first.x + pkt.second.l) &&
               (start.z >= pkt.first.z) && (start.z < pkt.first.z + pkt.second.h)) {
                advanced_eps.push_back(potential_ep);
                hit_bottom = false;
                break;
            }
            // check if the pkt is being blocked by other packets
            if(start.x >= pkt.first.x){
                bool is_blocked = false;
                for(auto j: ULD_blocking_boxes_y){
                    auto blocking_pkt = placement[j];
                    if(blocking_pkt.first.x == -1) continue; // although this should not happen
                    if(blocking_pkt.first.y > pkt.first.y + pkt.second.b) continue;
                    if(!((blocking_pkt.first.z + blocking_pkt.second.h > start.z) && (blocking_pkt.first.z <= pkt.first.z))) continue;
                    if(!((start.x >= blocking_pkt.first.x) && (start.x < blocking_pkt.first.x + blocking_pkt.second.l))) continue;
                    is_blocked = true;
                    break;
                }
                if(is_blocked) continue;
            }
            else if(start.z >= pkt.first.z){
                // give line by line 
                bool is_blocked = false;
                for(auto j: ULD_blocking_boxes_y){
                    auto blocking_pkt = placement[j];
                    if(blocking_pkt.first.x == -1) continue; // although this should not happen
                    if(blocking_pkt.first.y > pkt.first.y + pkt.second.b) continue;
                    if(!((blocking_pkt.first.x + blocking_pkt.second.l > start.x) && (blocking_pkt.first.x <= pkt.first.x))) continue;
                    if(!((start.z >= blocking_pkt.first.z) && (start.z < blocking_pkt.first.z + blocking_pkt.second.h))) continue;
                    is_blocked = true;
                    break;
                }
                if(is_blocked) continue;   
            }

            else{ // diagonal intersection
                bool is_blocked = false;
                for(auto j: ULD_blocking_boxes_y){
                    auto blocking_pkt = placement[j];
                    if(blocking_pkt.first.x == -1) continue; // although this should not happen
                    is_blocked = check_collision_2D(pkt.first.x, pkt.first.x + pkt.second.l, 
                                                    pkt.first.z, pkt.first.z + pkt.second.h,
                                                    blocking_pkt.first.x, blocking_pkt.first.x + blocking_pkt.second.l, 
                                                    blocking_pkt.first.z, blocking_pkt.first.z + blocking_pkt.second.h);
                    if(is_blocked) break;
                }
                if(is_blocked) continue;   
            }

            // add to the blocking pkts list
            ULD_blocking_boxes_y.insert(i);
            if(pkt.first.y + pkt.second.b < start.y){
                advanced_eps.push_back(potential_ep);
            }
        }
        ULD_blocking_boxes_y.clear();
        if(hit_bottom){
            coords potential_ep;
            potential_ep.box = start.box;
            potential_ep.x = start.x;
            potential_ep.y = 0;
            potential_ep.z = start.z;
            advanced_eps.push_back(potential_ep);
        }
        return;
}

void Solver::projection_neg_x_advanced(vector<coords> &advanced_eps, coords start, int self_pid){
    
        bool hit_bottom = true;
        for(auto i: ULD_sorted_x[start.box])
        {
            coords potential_ep;
            potential_ep.box = start.box;
            potential_ep.x = placement[i].first.x + placement[i].second.l;
            potential_ep.y = start.y;
            potential_ep.z = start.z;
    
            if(i == self_pid)
                continue;
            auto pkt = placement[i];
            if(pkt.first.x == -1) continue;
            if(pkt.first.y + pkt.second.b <= start.y) continue;
            if(pkt.first.z + pkt.second.h <= start.z) continue;
            if(pkt.first.x >= start.x) continue;
    
            // case 0: ray hits the object (no more projections)
            if((start.y >= pkt.first.y) && (start.y < pkt.first.y + pkt.second.b) &&
               (start.z >= pkt.first.z) && (start.z < pkt.first.z + pkt.second.h)) {
                advanced_eps.push_back(potential_ep);
                hit_bottom = false;
                break;
            }
            // check if the pkt is being blocked by other packets
            if(start.y >= pkt.first.y){
                bool is_blocked = false;
                for(auto j: ULD_blocking_boxes_x){
                    auto blocking_pkt = placement[j];
                    if(blocking_pkt.first.y == -1) continue; // although this should not happen
                    if(blocking_pkt.first.x > pkt.first.x + pkt.second.l) continue;
                    if(!((blocking_pkt.first.z + blocking_pkt.second.h > start.z) && (blocking_pkt.first.z <= pkt.first.z))) continue;
                    if(!((start.y >= blocking_pkt.first.y) && (start.y < pkt.first.y + pkt.second.b))) continue;
                    is_blocked = true;
                    break;
                }
                if(is_blocked) continue;
            }
            else if(start.z >= pkt.first.z){
                // give line by line 
                bool is_blocked = false;
                for(auto j: ULD_blocking_boxes_x){
                    auto blocking_pkt = placement[j];
                    if(blocking_pkt.first.y == -1) continue; // although this should not happen
                    if(blocking_pkt.first.x > pkt.first.x + pkt.second.l) continue;
                    if(!((blocking_pkt.first.y + blocking_pkt.second.b > start.y) && (blocking_pkt.first.y <= pkt.first.y))) continue;
                    if(!((start.z >= blocking_pkt.first.z) && (start.z < blocking_pkt.first.z + blocking_pkt.second.h))) continue;
                    is_blocked = true;
                    break;
                }
                if(is_blocked) continue;   
            }
            else{ // diagonal intersection
                bool is_blocked = false;
                for(auto j: ULD_blocking_boxes_x){
                    auto blocking_pkt = placement[j];
                    if(blocking_pkt.first.y == -1) continue; // although this should not happen
                    is_blocked = check_collision_2D(pkt.first.y, pkt.first.y + pkt.second.b, 
                                                    pkt.first.z, pkt.first.z + pkt.second.h,
                                                    blocking_pkt.first.y, blocking_pkt.first.y + blocking_pkt.second.b, 
                                                    blocking_pkt.first.z, blocking_pkt.first.z + blocking_pkt.second.h);
                    if(is_blocked) break;
                }
                if(is_blocked) continue;   
            }

            // add to the blocking pkts list
            ULD_blocking_boxes_x.insert(i);
            if(pkt.first.x + pkt.second.l < start.x){
                advanced_eps.push_back(potential_ep);
            }
        }
        ULD_blocking_boxes_x.clear();
        if(hit_bottom){
            coords potential_ep;
            potential_ep.box = start.box;
            potential_ep.x = 0;
            potential_ep.y = start.y;
            potential_ep.z = start.z;
            advanced_eps.push_back(potential_ep);
        }
        return;
}
void Solver::addEP2(int i)
{
    // ep.erase(pair<int,pii>(placement[i].first.x,pii(placement[i].first.y,placement[i].first.z)));
    // coords ob1;
    // auto p = placement[i];
    // // placement[i].first=def;
    // ob1.x=p.first.x+placement[i].second.l;
    // ob1.y=p.first.y;
    // ob1.z=p.first.z+p.second.h;
    // vector<pair<pair<int,int>,pair<int,int>>>faces,faces_checked;
    // For(j,i)
    // {
    //     faces.push_back(make_pair(make_pair(placement[j].first.x,placement[j].first.y),make_pair(placement[j].first.y+placement[j].second.b,placement[j].first.z+placement[j].second.h)));
    // }
    // sort(faces.begin(),faces.end(),check);
    // For(j,i)
    // {
    //     int x1=faces[j].first.first;
    //     // int y1=faces[j].second.first;
    //     int z1=faces[j].second.second;
    //     if(ob1.z<z1 || ob1.x>x1)
    //     continue;
    //     if(((ob1.y>=faces[j].first.second) && (ob1.y<=faces[j].second.first))==false)
    //     continue;
    //     int t=0;
    //     for(auto yy:placement)
    //     {
    //         int x=yy.first.x;
    //         int y=yy.first.y;
    //         int z=yy.first.z;
    //         if(ob1.x<x && x<x1 && z1>yy.first.z && z1<yy.first.z+yy.second.h)
    //         t=1;
    //     }
    //     // if(t==0)
    //     if(t==0)
    //     {
    //         coords d;
    //         d.x=ob1.x;
    //         d.y=ob1.y;
    //         d.z=faces[j].second.second;
    //         auto r =getResidueSpace(d);
    //         if(r.first*r.second.first*r.second.second!=0)ep[convertCoords(d)] = r;
    //     }
    //     // faces_checked.push_back(faces[j]);
    // }
    // cout << placement[i].first.box << " ";
    coords ob1, ob2, ob3;
    auto p = placement[i];
    // p.first = def;
    int uid = p.first.box;
    ob1.box = uid;
    ob1.x = p.first.x;
    ob1.y = p.first.y + p.second.b;
    ob1.z = p.first.z + p.second.h;

    ob2.box = uid;
    ob2.x = p.first.x + p.second.l;
    ob2.y = p.first.y;
    ob2.z = p.first.z + p.second.h;

    ob3.box = uid;
    ob3.x = p.first.x + p.second.l;
    ob3.y = p.first.y + p.second.b;
    ob3.z = p.first.z;
    // cout << ob1.x << " " << ob1.y << " " << ob1.z << endl;
    // cout << ob2.x << " " << ob2.y << " " << ob2.z << endl;
    // cout << ob3.x << " " << ob3.y << " " << ob3.z << endl;
    // cout << endl;
    
    vector<coords> advanced_eps;
    projection_neg_y_advanced(advanced_eps, ob1, i);
    projection_neg_z_advanced(advanced_eps, ob1, i);

    projection_neg_x_advanced(advanced_eps, ob2, i);
    projection_neg_z_advanced(advanced_eps, ob2, i);

    projection_neg_x_advanced(advanced_eps, ob3, i);
    projection_neg_y_advanced(advanced_eps, ob3, i);
    cout << advanced_eps.size() << " ";

    for(auto x: advanced_eps){
        auto r = getResidueSpace(x);
        if((r.first > RESIDUE_THRESHOLD) and (r.second.first > RESIDUE_THRESHOLD) and (r.second.second) > RESIDUE_THRESHOLD)
            ep[convertCoords(x)] = r;
    }
    placement[i] = p; 
    
}
void Solver::addEP(int i)
{
    // ep.erase(pair<int, pair<int, pii>>(placement[i].first.box, pair<int, pii>(placement[i].first.x, pii(placement[i].first.y, placement[i].first.z))));
    coords ob1;
    auto p = placement[i];
    placement[i].first = def;
    ob1.box = p.first.box;
    ob1.x = p.first.x;
    ob1.y = p.first.y + placement[i].second.b;
    ob1.z = p.first.z + p.second.h;
    auto t = beamprojectYNeg(ob1);
    auto r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    t = beamprojectZNeg(ob1);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    coords ob2;
    ob2.box = p.first.box;
    ob2.x = p.first.x + p.second.l;
    ob2.y = p.first.y;
    ob2.z = p.first.z + p.second.h;
    t = beamprojectXNeg(ob2);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    t = beamprojectZNeg(ob2);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    coords ob3;
    ob3.box = p.first.box;
    ob3.x = p.first.x + p.second.l;
    ob3.y = p.first.y + p.second.b;
    ob3.z = p.first.z;
    t = beamprojectXNeg(ob3);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second >= RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    t = beamprojectYNeg(ob3);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    placement[i] = p;
}
void Solver::updateMaxBound(int i)
{
    int b = placement[i].first.box;
    ULDl[b].maxBound.x = max(ULDl[b].maxBound.x, placement[i].first.x + placement[i].second.l);
    ULDl[b].maxBound.y = max(ULDl[b].maxBound.y, placement[i].first.y + placement[i].second.b);
    ULDl[b].maxBound.z = max(ULDl[b].maxBound.z, placement[i].first.z + placement[i].second.h);
}
void Solver::updateResidue(int i)
{
    // update residual space also to be done
    auto it = ep.lower_bound(convertCoords(placement[i].first));
    int b = placement[i].first.box;
    while (it != ep.end() and it->first.first == b)
    {
        auto p = *it;
        coords e;
        e.box = p.first.first;
        e.x = p.first.second.first;
        e.y = p.first.second.second.first;
        e.z = p.first.second.second.second;
        int t = XBeamIntersectionWithBox(e, i);
        if (t != -1)
            p.second.first = min(p.second.first, t);
        t = YBeamIntersectionWithBox(e, i);
        if (t != -1)
            p.second.second.first = min(p.second.second.first, t);
        t = ZBeamIntersectionWithBox(e, i);
        if (t != -1)
            p.second.second.second = min(p.second.second.second, t);
        it++;
    }
}
int Solver::XBeamIntersectionWithBox(coords start, int ind)
{
    int r = INF;
    int b = start.box;
    For(i, 2) For(j, 2)
    {
        auto p = start;
        p.y = min(p.y + i, ULDl[b].dim.b - 1);
        p.z = min(p.z + j, ULDl[b].dim.h - 1);
        int t = XRayIntersectionWithBox(p, ind);
        if (t != -1)
        {
            r = min(r, t);
        }
    }
    return r == INF ? -1 : r;
}
int Solver::YBeamIntersectionWithBox(coords start, int ind)
{
    int r = INF;
    int b = start.box;
    For(i, 2) For(j, 2)
    {
        auto p = start;
        p.x = min(p.x + i, ULDl[b].dim.l - 1);
        p.z = min(p.z + j, ULDl[b].dim.h - 1);
        int t = YRayIntersectionWithBox(p, ind);
        if (t != -1)
        {
            r = min(r, t);
        }
    }
    return r == INF ? -1 : r;
}
int Solver::ZBeamIntersectionWithBox(coords start, int ind)
{
    int r = INF;
    int b = start.box;
    For(i, 2) For(j, 2)
    {
        auto p = start;
        p.x = min(p.x + i, ULDl[b].dim.l - 1);
        p.y = min(p.y + j, ULDl[b].dim.b - 1);
        int t = ZRayIntersectionWithBox(p, ind);
        if (t != -1)
        {
            r = min(r, t);
        }
    }
    return r == INF ? -1 : r;
}
int Solver::XRayIntersectionWithBox(coords start, int ind)
{
    if (start.y < placement[ind].first.y + placement[ind].second.b and start.y > placement[ind].first.y and start.z < placement[ind].first.z + placement[ind].second.h and start.z > placement[ind].first.z and start.x < placement[ind].first.x + placement[ind].second.l and start.box == placement[ind].first.box)
        return placement[ind].first.x;
    return -1;
}
int Solver::YRayIntersectionWithBox(coords start, int ind)
{
    if (start.x < placement[ind].first.x + placement[ind].second.l and start.x > placement[ind].first.x and start.z < placement[ind].first.z + placement[ind].second.h and start.z > placement[ind].first.z and start.y < placement[ind].first.y + placement[ind].second.b and start.box == placement[ind].first.box)
        return placement[ind].first.y;
    return -1;
}
int Solver::ZRayIntersectionWithBox(coords start, int ind)
{
    if (start.x < placement[ind].first.x + placement[ind].second.l and start.x > placement[ind].first.x and start.y < placement[ind].first.y + placement[ind].second.b and start.y > placement[ind].first.y and start.z < placement[ind].first.z + placement[ind].second.h and start.box == placement[ind].first.box)
        return placement[ind].first.z;
    return -1;
}
coords Solver::rayProjectXNeg(coords start)
{
    int x = 0;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.y <= placement[i].first.y + placement[i].second.b and start.y >= placement[i].first.y and start.z <= placement[i].first.z + placement[i].second.h and start.z >= placement[i].first.z and start.x >= placement[i].first.x)
            x = max(x, placement[i].first.x + placement[i].second.l);
    }
    start.x = min(start.x, x);
    return start;
}
coords Solver::rayProjectYNeg(coords start)
{
    int x = 0;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.x <= placement[i].first.x + placement[i].second.l and start.x >= placement[i].first.x and start.z <= placement[i].first.z + placement[i].second.h and start.z >= placement[i].first.z and start.y >= placement[i].first.y)
            x = max(x, placement[i].first.y + placement[i].second.b);
    }
    start.y = min(start.y, x);
    return start;
}
coords Solver::rayProjectZNeg(coords start)
{
    int x = 0;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.x <= placement[i].first.x + placement[i].second.l and start.x >= placement[i].first.x and start.y <= placement[i].first.y + placement[i].second.b and start.y >= placement[i].first.y and start.z >= placement[i].first.z)
            x = max(x, placement[i].first.z + placement[i].second.h);
    }
    start.z = min(start.z, x);
    return start;
}
coords Solver::rayProjectXPos(coords start)
{
    int x = ULDl[start.box].dim.l;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.y < placement[i].first.y + placement[i].second.b and start.y > placement[i].first.y and start.z < placement[i].first.z + placement[i].second.h and start.z > placement[i].first.z and start.x < placement[i].first.x + placement[i].second.l)
            x = min(x, placement[i].first.x);
    }
    start.x = max(start.x, x);
    return start;
}
coords Solver::rayProjectYPos(coords start)
{
    int x = ULDl[start.box].dim.b;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.x < placement[i].first.x + placement[i].second.l and start.x > placement[i].first.x and start.z < placement[i].first.z + placement[i].second.h and start.z > placement[i].first.z and start.y < placement[i].first.y + placement[i].second.b)
            x = min(x, placement[i].first.y);
    }
    start.y = max(start.y, x);
    return start;
}
coords Solver::rayProjectZPos(coords start)
{
    int x = ULDl[start.box].dim.h;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.x < placement[i].first.x + placement[i].second.l and start.x > placement[i].first.x and start.y < placement[i].first.y + placement[i].second.b and start.y > placement[i].first.y and start.z < placement[i].first.z + placement[i].second.h)
            x = min(x, placement[i].first.z);
    }
    start.z = max(start.z, x);
    return start;
}
bool Solver::checkGravity(coords e, Box b)
{
    // cout << surfaces.size();
    if(e.z==0)
    return false;
    // return false;
    // sort(surfaces.begin(),surfaces.end());
    // auto start=surfaces.lower_bound(make_pair(e.z,make_pair(make_pair(0,0),make_pair(0,0))));
    // auto end=surfaces.lower_bound(make_pair(e.z,make_pair(make_pair(3000,3000),make_pair(3000,3000))));
    // for(auto x:surfaces)
    // cout << x.first << " ";
    // cout << "\n";
    pair<int,int>x1_min=make_pair(e.x,e.y),x1_max={e.x+b.l,e.y+b.b};
    int flag=0;
    int c=0;
    for(auto i:surfaces[e.box])
    {
        if((i).first!=e.z)
        continue;
        c++;
    }
    if(c==0)
    return true;
    for(auto i:surfaces[e.box])
    {
        if(i.first!=e.z)
        continue;
        pair<int,pair<pair<int,int>,pair<int,int>>>p=i;
        pair<int,int>x2_min=make_pair(p.second.first.first,p.second.first.second),x2_max={p.second.second.first,p.second.second.second};
        if(x1_max.first>x2_min.first && x1_min.first<x2_max.first && x1_max.second>x2_min.second && x1_min.second<x2_max.second)
        flag=1;
    }
    if(flag==1)
    return false;
    return true;
}
void Solver:: gravity_pull(int i)
{
    pair<int,int>x1_min=make_pair(placement[i].first.x,placement[i].first.y),x1_max={placement[i].first.x+placement[i].second.l,placement[i].first.y+placement[i].second.b};
    int m=0;
    for(auto x:surfaces[placement[i].first.box])
    {
        pair<int,pair<pair<int,int>,pair<int,int>>>p=x;
        pair<int,int>x2_min=make_pair(p.second.first.first,p.second.first.second),x2_max={p.second.second.first,p.second.second.second};
        if(x1_max.first>x2_min.first && x1_min.first<x2_max.first && x1_max.second>x2_min.second && x1_min.second<x2_max.second && x.first<=placement[i].first.z)
        m=max(m,x.first);
    }
    placement[i].first.z=m;
}
// void Solver::gravity_pull(int i)
// {
//     auto x1_min = make_pair(placement[i].first.x, placement[i].first.y);
//     auto x1_max = make_pair(placement[i].first.x + placement[i].second.l, placement[i].first.y + placement[i].second.b);
    
//     int m = 0;
//     for (auto x : surfaces)
//     {
//         auto p = x;
//         auto x2_min = make_pair(p.second.first.first, p.second.first.second);
//         auto x2_max = make_pair(p.second.second.first, p.second.second.second);

//         bool is_overlapping = x1_max.first > x2_min.first &&
//                               x1_min.first < x2_max.first &&
//                               x1_max.second > x2_min.second &&
//                               x1_min.second < x2_max.second;

//         if (is_overlapping && p.first <= placement[i].first.z)
//         {
//             m = max(m, p.first);
//         }
//     }

//     placement[i].first.z = m;
//     // c+=1-checkGravity(placement[i].first,placement[i].second);
// }

// double weightz = 0.2;

int residueFunc(coords c, Box b,Solver* s){
    int r= 0;
    r+=(1LL*(s->ULDHasPriority[c.box])*100000000000LL)*b.isPriority;
    float relativeDifference =(s->ep[convertCoords(c)].first - b.l)/1.0/s->ep[convertCoords(c)].first+(s->ep[convertCoords(c)].second.first - b.b)/1.0/s->ep[convertCoords(c)].second.first+weightz*(s->ep[convertCoords(c)].second.second - b.h)/1.0/s->ep[convertCoords(c)].second.second;
    relativeDifference*=1000000;
//    float relativeDifference =(s->ep[convertCoords(c)].first - b.l)+(s->ep[convertCoords(c)].second.first - b.b)+0.1*(s->ep[convertCoords(c)].second.second - b.h);
    r+=relativeDifference;
    return r;
}


void ScoredSolver::solve(){
    // Initial Solve
    insertionCounter.assign(data.size()+5, 0);
    #ifndef GENETIC
    #ifdef OLD_SORT
    sort(data.begin(), data.end(), this->sorter.val);
    #endif
    #ifndef OLD_SORT
    this->sorter.val(data);
    #endif
    #endif
    #ifdef DATA_ORDERING
    cout << "Data ordering:" << endl;
    for(auto it: data){
        // cout << it.ID << "," << score[it.ID] << " ";
        cout << it.ID << " ";
    }
    cout << endl;
    #endif
    For(i, ULDl.size()) ep[pair<int, pair<int, pii>>(i, pair<int, pii>(0, pii(0, 0)))] = pair<int, pii>(ULDl[i].dim.l, pii(ULDl[i].dim.b, ULDl[i].dim.h));
    For(i, data.size())
    {
        // Construct Economy Package and Box Map
        boxMap[data[i].ID] = &data[i];
        if(!data[i].isPriority){
            
            // cout << "Box ID: " << data[i].ID << endl;
            economyPackages.insert(data[i].ID);
        }

        Box b = data[i];
        pair<int, pair<coords, Box>> best;
        best.first = -INF;
        vector<Box> perms(6);
        perms[0].l = b.l;
        perms[0].b = b.b;
        perms[0].h = b.h; 
        perms[1].l = b.l;
        perms[1].h = b.b;
        perms[1].b = b.h;
        perms[2].l = b.b;
        perms[2].b = b.l;
        perms[2].h = b.h;
        perms[3].l = b.b;
        perms[3].b = b.h;
        perms[3].h = b.l;
        perms[4].l = b.h;
        perms[4].b = b.b;
        perms[4].h = b.l;
        perms[5].l = b.h;
        perms[5].b = b.l;
        perms[5].h = b.b;
        for (Box p : perms)
            for (auto x : ep)
            {
                // check if it can fit
                coords e;
                e.box = x.first.first;
                e.x = x.first.second.first;
                e.y = x.first.second.second.first;
                e.z = x.first.second.second.second;
                if(checkCollision(e,p))
                    continue;
                p.isPriority = b.isPriority;
                int score = this->merit.val(e, p, this);
                if (best.first < score)
                    best = pair<int, pair<coords, Box>>(score, pair<coords, Box>(e, p));
            }
        // update vals
        if (best.first == -INF)
            continue;
        insertionCounter[data[i].ID]+=1;
        if (data[i].isPriority){
            ULDHasPriority[best.second.first.box] = true;
            lastInsertionSet.insert(data[i].ID);
            lastInsertion.push_back(data[i].ID);
        }
        else{
            lastInsertionSet.insert(data[i].ID);
            lastInsertion.push_back(data[i].ID);
        }
        placement[i] = best.second;
        ULDPackages[best.second.first.box].insert(i);
        // ULDPackages[best.second.first.box].insert(i);
        ULD_sorted_x[best.second.first.box].insert(i);
        ULD_sorted_y[best.second.first.box].insert(i);
        ULD_sorted_z[best.second.first.box].insert(i);
        // int pp=placement[i].first.z;
        gravity_pull(i);
        // if(pp!=placement[i].first.z)
        // c++;
        addEP2(i);
        // addEP(i);
        update(i);
        // if(placement[i].second==data[i])
        // if(checkGravity(placement[i].first, placement[i].second))
        // printf("error\n");
    }

    // Initialising the scores
    // double k = 2.0;
    // for(auto i: economyPackages){
    //     // if(lastInsertionSet.find(i) == lastInsertionSet.end()){
    //     if(0){
    //         score[i] = k*boxMap[i]->cost;
    //     }
    //     else{
    //         score[i] = boxMap[i]->cost;
    //     }
    // }
    cout << "Base Solution Cost: " << this->cost() << endl;
    // #define DATA_ORDERING
    #ifdef DATA_ORDERING
    cout << "Data ordering:" << endl;
    for(auto it: data){
        // cout << it.ID << "," << score[it.ID] << " ";
        cout << it.ID << " ";
    }
    cout << endl;
    #endif
    bestCost = this->cost();
    // cout << bestCost;
    // return;
    bestSolution = lastInsertion;
    // optimize(0);
    // writeToFile("other_result.csv");
    // cout << "New Cost: " << this->cost() << endl;
    // assert(this->cost() == bestCost);
    // return;
    // costDensityOptimize();
    // return;
    bestSolutionSwaps(50);
    return;
    int lastChangeIter = -1;
    int reinitializeIter = 100, noChangeThreshold = 10;
    int solverTime = static_cast<int>(time(nullptr));
    for(int i = 1; i <= iterations; i++){
        cout << "Iteration " << i << " started" << endl;
        if(i%reinitializeIter == 1 || i - lastChangeIter >= noChangeThreshold){
            cout << "Reinitializing at iteration " << i << endl;
            reinitialize(true, 2.0, 25);
            lastChangeIter = i;
        }
        else{
            update_scores(i);
        }
        // Solver new_solver(this->sorter, this->merit, this->data, this->originalUldList);
        // new_solver.solve();
        // cout << "New solver costs me " << new_solver.cost() << endl;
        optimize(i);
        cout << "Iteration " << i << " had a cost " << this->cost() << endl;
        if(this->cost() > bestCost){
            cout << "Found new solution at iteration " << i << " with cost " << this->cost() << endl;
            stringstream ss;
            ss << "result_" << i << "_" << solverTime << ".txt";
            this->writeToFile(ss.str());
            bestCost = this->cost();
            lastChangeIter = i;
            bestSolution = lastInsertion;
            cout << "Updated base solution to " << endl;
            for(auto it: bestSolution){
                cout << it << " ";
            }
        }
    }
}

void ScoredSolver::arrangeDataFromIDVector(vector<int> idVector){
    // Construct BoxMap
    // boxMap.clear();
    vector<Box> boxVector(data.size() +  10);
    // boxMap.assign(data.size()+10, nullptr);
    for(auto it: data){
        // cout << it.ID << ",";
        boxVector[it.ID] = it;
        // cout << boxMap[it.ID]->ID << " ";
    }
    for(auto  it: boxMap){
        // cout << 
        // cout << it.first << " " << it.second->ID << endl;
    }
    vector<Box> newData;
    // cout << endl;
    for(auto it: idVector){
        // cout << it << "," << boxVector[it].ID << " ";
        newData.push_back(boxVector[it]);
    }
    data = newData;
}
/*
* Trying to optimize the solution by checking the cost/volume of the packets left out.
*/
void ScoredSolver::costDensityOptimize(){
    set<int> bestSolutionSet = set<int>(bestSolution.begin(), bestSolution.end());
    set<pair<double, int>, greater<pair<double, int>>> costDensity;
    for(auto it: economyPackages){
        costDensity.insert({1.0*boxMap[it]->cost/(boxMap[it]->l*boxMap[it]->b*boxMap[it]->h), it});
    }
    int economyPackageCounter = 0, economyPackageCount = 0;
    for(auto it: bestSolution){
        if(!boxMap[it]->isPriority){
            economyPackageCounter++;
        }
    }
    int packageRanking = 0;
    for(auto it: costDensity){
        packageRanking++;
        if(bestSolutionSet.find(it.second) == bestSolutionSet.end()){
            // if(economyPackageCount < economyPackageCounter){

                bool t = true;
                for(auto it2: bestSolution){
                    if(boxMap[it2]->isPriority){
                        continue;
                    }
                    if(1.0*boxMap[it2]->cost/(boxMap[it2]->l*boxMap[it2]->b*boxMap[it2]->h) < it.first && boxMap[it2]->cost < boxMap[it.second]->cost){
                        if(t){
                            cout << "Density Rank: " << packageRanking << " Density: " << it.first << " ID: " << it.second << " Cost: " << boxMap[it.second]->cost << " Volume: " << boxMap[it.second]->l*boxMap[it.second]->b*boxMap[it.second]->h << endl;
                            t = false;
                        }
                        // Stricter Inequality:
                        int dimensions_1[3] = {boxMap[it.second]->l, boxMap[it.second]->b, boxMap[it.second]->h};
                        int dimensions_2[3] = {boxMap[it2]->l, boxMap[it2]->b, boxMap[it2]->h};
                        sort(dimensions_1, dimensions_1+3);
                        sort(dimensions_2, dimensions_2+3);
                        bool stricter = true;
                        if(!stricter || dimensions_1[0] <= dimensions_2[0] && dimensions_1[1] <= dimensions_2[1] && dimensions_1[2] <= dimensions_2[2]){
                            cout << "   - Can replaced " << it2 << " with cost density " << 1.0*boxMap[it2]->cost/(boxMap[it2]->l*boxMap[it2]->b*boxMap[it2]->h) << " and cost " << boxMap[it2]->cost << " and volume " << boxMap[it2]->l * boxMap[it2]->b * boxMap[it2]->h << endl;
                        }
                        // cout << "   - Can be replaced with " << it2 << " with cost density " << 1.0*boxMap[it2]->cost/(boxMap[it2]->l*boxMap[it2]->b*boxMap[it2]->h) << " and cost " << boxMap[it2]->cost << " and volume " << boxMap[it2]->l * boxMap[it2]->b * boxMap[it2]->h << endl;
                    }
                }
                // cout << "Found better cost density package which hasn't even been inserted yet with cost density " << it.first << " and ID " << it.second << endl;
                // cout << "Current economy package count: " << economyPackageCount << endl;   
                // cout << "Package is ranked at: " << packageRanking << endl;
            // }
            
        }
        else{
            economyPackageCount++;
        }
    }

}
/// @brief 
/// @param swaps 
void ScoredSolver::bestSolutionSwaps(int swaps){
    int lastPriority = 0;
    for(lastPriority = 0; lastPriority < bestSolution.size(); lastPriority++){
        if(!boxMap[bestSolution[lastPriority]]->isPriority){
            lastPriority--;
            break;
        }
    }
    Sorter emptySorter;
    emptySorter.val = [](vector<Box> &data){
        return;
    };
    // Constructed Solution: 
    vector<Box> constructedSolution;
    for(int i = 0; i < bestSolution.size(); i++){
        constructedSolution.push_back(*boxMap[bestSolution[i]]);
    }
    set<int> bestSolutionSet = set<int>(bestSolution.begin(), bestSolution.end());
    vector<Box> nonInsertedObjects;
    // int firstNonInsertedIndex = constructedSolution.size();
    for(auto it: economyPackages){
        if(bestSolutionSet.find(it) == bestSolutionSet.end()){
            // constructedSolution.push_back(*boxMap[it]);
            nonInsertedObjects.push_back(*boxMap[it]);
        }
    }
    this->sorter.val(nonInsertedObjects);
    for(auto it: nonInsertedObjects){
        constructedSolution.push_back(it);
    }
    

    FILE* file = freopen("analysis.txt", "w", stdout);
    for(int i = 0; i < constructedSolution.size()-50; i++){
        int j;
        // if(swaps != -1){
        //     j = max(static_cast<int>(0), i-swaps);
        // }
        // else{
        //     j = 0;
        // }
        // if(i % 50 == 0){
        //     cout.flush();
        // }
        cout << "Swapping " << i << "\n";
        cout.flush();
        for(j = max(static_cast<int>(0), i-50); j < i; j++){
            // cout << "Swapping " << i << " " << j << endl;
            swap(constructedSolution[i], constructedSolution[j]);
            Solver sr(emptySorter, this->merit, constructedSolution, this->originalUldList);
            sr.solve();
            // cout << "Found a solution at swapping " << i << " " << j << " with cost " << sr.cost() << endl;
            
            if(sr.cost() > bestCost){
                // Maybe update best solution
                bestCost = sr.cost();
                cout << "swapping " << i << " " << j << " as " << constructedSolution[i].ID << " " << constructedSolution[j].ID << " with cost " << sr.cost() << "\n";
                this->data = sr.data;
                sr.writeToFile("best_solution.txt");
            }
            else{
                swap(constructedSolution[i], constructedSolution[j]);
            }
            
        }
    }
    cout.flush();
    fclose(file);

    
    
    
    


}


// void ScoredSolver::insertedSwap(int _numSwaps){
//     int numSwaps = _numSwaps;
//     for(int i = 0; i < numSwaps; i++){
//         mt19937 mt(time(nullptr));
//         int idx, idy;
//         do{
//             idx = mt()%bestSolution.size();
//             idy = mt()%bestSolution.size();
//         } while(idx == idy);
//         swap(score[bestSolution[idx]], score[bestSolution[idy]]);
//     }


// }

void ScoredSolver::reinitialize(bool _swap, double k, int num_swap){
    set<int> bestSolutionSet(bestSolution.begin(), bestSolution.end());
    vector<int> unselectedPackages;
    for(auto it: economyPackages){
        if(bestSolutionSet.find(it) != bestSolutionSet.end()){
            score[it] = 1.0*boxMap[it]->cost;
        }
        else{
            unselectedPackages.push_back(it);
            score[it] = 0;
            // score[it] = boxMap[it]->cost;
        }
    }
    
    vector<int> economyPackagesVec(economyPackages.begin(), economyPackages.end());
    if(_swap){
        for(int i = 0; i < num_swap; i++){
            mt19937 mt(time(nullptr));
            #define SWAP_DIFFERENT
            #ifndef SWAP_DIFFERENT
            int idx, idy;
            do{
                idx = mt()%economyPackagesVec.size();
                idy = mt()%economyPackagesVec.size();
            } while(idx == idy);
            swap(score[economyPackagesVec[idx]], score[economyPackagesVec[idy]]);
            #endif
            #ifdef SWAP_DIFFERENT
            int idx, idy;
            do{
                idx = mt()%bestSolution.size();
                idy = mt()%unselectedPackages.size();
            } while(idx == idy);
            swap(score[bestSolution[idx]], score[unselectedPackages[idy]]);
            #endif

        }
    }
}

/*
* i = number of iterations completed
*/
void ScoredSolver::update_scores(int i){
    
    pair<double, int> worst_loaded = {100000000.0, -1}, best_unloaded = {0.0, -1};
    for(auto id : economyPackages){
        if(lastInsertionSet.find(id) == lastInsertionSet.end()){
            double theta = (1.0*boxMap[id]->cost)/(boxMap[id]->l * boxMap[id]->b * boxMap[id]->h * (1+i-insertionCounter[id]));
            // cout << "Theta: "<< theta << " " << id << endl;
            if(theta > best_unloaded.first){
                best_unloaded = {theta, id};
            }

        }
        else{
            double myu = (1.0*boxMap[id]->cost)/(boxMap[id]->l * boxMap[id]->b * boxMap[id]->h * (1+insertionCounter[id]));
            if(myu < worst_loaded.first){
                worst_loaded = {myu, id};
            }
        }
    }
    // Found highest myu and theta
    // cout << "Reached here" << endl;
    // cout.flush();
    if(worst_loaded.second != -1 && best_unloaded.second != -1){
        score[worst_loaded.second] *= (0.5);
        score[best_unloaded.second] *= (1.5);
        
        cout << "Score of "<< best_unloaded.second << ":" << score[best_unloaded.second] << endl;
        swap(score[worst_loaded.second], score[best_unloaded.second]);
        cout << "Swapped " << worst_loaded.second << " and " << best_unloaded.second << "\n";
        cout << "Score of "<< best_unloaded.second << ":" << score[best_unloaded.second] << endl;
    }
    else{
        cout << "No possible swaps " << worst_loaded.second << " " << best_unloaded.second << endl;
        return;
    }
}


// void ScoredSolver::cachedSolve(){

// }

void ScoredSolver::optimize(int _iter){
    // Re-Setup the solver
    placement.clear();
    ULDPackages.clear();
    surfaces.clear();
    ULDHasPriority.clear();
    lastInsertion.clear();
    lastInsertionSet.clear();
    def.x = def.y = def.z = def.box = -1;
    Box def_;
    def_.l = def_.b = def_.h = -1;
    ULDl = this->originalUldList;
    placement.assign(data.size(), pair<coords, Box>(def, def_));
    ULDPackages.assign(ULDl.size(), set<int>());
    ULD_sorted_x.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_x));
    ULD_sorted_y.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_y));
    ULD_sorted_z.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_z));
    ULD_blocking_boxes_x = set<int, function<bool(const int &, const int &)>>(compare_x_base);
    ULD_blocking_boxes_y = set<int, function<bool(const int &, const int &)>>(compare_y_base);
    ULD_blocking_boxes_z = set<int, function<bool(const int &, const int &)>>(compare_z_base);
    surfaces.assign(ULDl.size(), set<pair<int,pair<pair<int,int>,pair<int,int>>>>());
    ULDHasPriority.assign(ULDl.size(), false);
    ep.clear();

    // sort(data.begin(), data.end(), this->sorter.val);
    #define SORT_WITH_MARKS
    #ifdef SORT_WITH_MARKS
    vector<int> mark(data.size()+5);
    for(int i =0; i != data.size(); i++){
        for(int j = 0; j != data.size(); j++){
            if(data[i].isPriority || data[j].isPriority)continue;
            if(data[i].cost < data[j].cost)continue;
            vector<int> perm_i = {data[i].l, data[i].b, data[i].h}; sort(perm_i.begin(), perm_i.end());
            vector<int> perm_j = {data[j].l, data[j].b, data[j].h}; sort(perm_j.begin(), perm_j.end());
            if(perm_i[0] <= perm_j[0] && perm_i[1] <= perm_j[1] && perm_i[2] <= perm_j[2]){
                if(!(perm_i[0] == perm_j[0] && perm_i[1] == perm_j[1] && perm_i[2] == perm_j[2]) && data[i].cost == data[j].cost){ // Ensure they don't correspond to the samme box either
                    mark[data[j].ID]++;
                    break;
                }
            }
        }
    }
    
    sort(data.begin(), data.end(), [&](Box a,Box b){
        if(b.isPriority and (not a.isPriority))return false;
        if(a.isPriority and (not b.isPriority))return true;
        // if(mark[a.ID] != mark[b.ID]) return mark[a.ID] < mark[b.ID];
        // if(a.isPriority and b.isPriority)return false;
        // if(a.cost!=b.cost)return a.cost>b.cost;
        int vol1 = a.l*a.b*a.h, vol2 = b.l*b.b*b.h;
        
            // double power_fac = 1.0;
        double fac1 = pow(score[a.ID], power_fac)/(vol1*1.0), fac2 = pow(score[b.ID], power_fac)/(vol2*1.0);
        if(score[a.ID]!=score[b.ID])return fac1 > fac2;

        // if(score[a.ID]!=score[b.ID])return score[a.ID]>score[b.ID];
        if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
        return a.l*a.b*a.h > b.l*b.b*b.h;
    });
    #endif
    #ifndef SORT_WITH_MARKS
    sort(data.begin(), data.end(), [&](Box a, Box b){
        if (a.isPriority && !b.isPriority)
            return true;
        if (!a.isPriority && b.isPriority)
            return false;
        // if(a.cost != b.cost) return a.cost > b.cost;

        if(score[a.ID]!=score[b.ID])return score[a.ID]>score[b.ID];
        if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
        return a.l*a.b*a.h > b.l*b.b*b.h;
    });
    #endif

    // this->sorter.val(data);
    #ifdef DATA_ORDERING
    cout << "Data ordering:" << endl;
    for(auto it: data){
        // cout << it.ID << "," << score[it.ID] << " ";
        cout << it.ID << " ";
    }
    cout << endl;
    #endif
    For(i, ULDl.size()) ep[pair<int, pair<int, pii>>(i, pair<int, pii>(0, pii(0, 0)))] = pair<int, pii>(ULDl[i].dim.l, pii(ULDl[i].dim.b, ULDl[i].dim.h));
    For(i, data.size())
    {
        // Construct Economy Package and Box Map
        Box b = data[i];
        pair<int, pair<coords, Box>> best;
        best.first = -INF;
        vector<Box> perms(6);
        perms[0].l = b.l;
        perms[0].b = b.b;
        perms[0].h = b.h; 
        perms[1].l = b.l;
        perms[1].h = b.b;
        perms[1].b = b.h;
        perms[2].l = b.b;
        perms[2].b = b.l;
        perms[2].h = b.h;
        perms[3].l = b.b;
        perms[3].b = b.h;
        perms[3].h = b.l;
        perms[4].l = b.h;
        perms[4].b = b.b;
        perms[4].h = b.l;
        perms[5].l = b.h;
        perms[5].b = b.l;
        perms[5].h = b.b;
        for (Box p : perms)
            for (auto x : ep)
            {
                // check if it can fit
                coords e;
                e.box = x.first.first;
                e.x = x.first.second.first;
                e.y = x.first.second.second.first;
                e.z = x.first.second.second.second;
                if(checkCollision(e,p))
                    continue;
                p.isPriority = b.isPriority;
                int scores = this->merit.val(e, p, this);
                if (best.first < scores)
                    best = pair<int, pair<coords, Box>>(scores, pair<coords, Box>(e, p));
            }
        // update vals
        if (best.first == -INF)
            continue;
        insertionCounter[data[i].ID]+=1;
        if (data[i].isPriority){
            ULDHasPriority[best.second.first.box] = true;
        }
        else{
            lastInsertionSet.insert(data[i].ID);
            lastInsertion.push_back(data[i].ID);
        }
        placement[i] = best.second;
        ULDPackages[best.second.first.box].insert(i);
        ULD_sorted_x[best.second.first.box].insert(i);
        ULD_sorted_y[best.second.first.box].insert(i);
        ULD_sorted_z[best.second.first.box].insert(i);
        // int pp=placement[i].first.z;
        gravity_pull(i);
        // if(pp!=placement[i].first.z)
        // c++;
        addEP2(i);
        // addEP(i);
        update(i);
        // if(placement[i].second==data[i])
        // if(checkGravity(placement[i].first, placement[i].second))
        // printf("error\n");
    }
}