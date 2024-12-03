#include "solver.h"

using namespace std;

bool check(const pair<pair<int, int>, pair<int, int>> &a, const pair<pair<int, int>, pair<int, int>> &b)
{ // custom comparator to sort
    int p = a.second.second;
    int q = b.second.second;
    return (p > q); // returns box with higher height
}
Solver::Solver(Sorter sorter_, Merit merit_, vector<Box> boxes, vector<Uld> ULD_) // constructor
{
    this->merit = merit_;
    this->sorter = sorter_;
    this->data = boxes;
    def.x = def.y = def.z = def.box = -1;
    Box def_;
    def_.l = def_.b = def_.h = -1;

    // initialization
    this->placement.assign(boxes.size(), pair<coords, Box>(def, def_));
    ULDl = ULD_;
    ULDPackages.assign(ULDl.size(), set<int>());
    surfaces.assign(ULDl.size(), set<pair<int, pair<pair<int, int>, pair<int, int>>>>());
    ULDHasPriority.assign(ULDl.size(), false);
}

int Solver::cost()
{
    int totalcost = 0; // to store total cost of solution
    set<int> priorityShipments; // to store ULDs containing priority packages
    for (int i = 0; i < data.size(); i++)
    {
        if (placement[i].first.x == -1) // if the package is not placed in any ULD
        {
            totalcost += data[i].cost; // add the cost of economy package to the toal
        }
        else
        {
            if (data[i].isPriority)
                priorityShipments.insert(placement[i].first.box); // insert the ULD-ID into the set "priorityShipments"
        }
    }
    totalcost += priorityShipments.size() * PRIORITY_ULD_COST; //  add the cost of priority packages in different ULDs to the total cost
    return totalcost;
}

bool Solver::checkCollision(coords e, Box b)
{
    // checks collision of prespective packages with all other packages of same ULD along with weight constraint of the ULD
    if (e.x + b.l > ULDl[e.box].dim.l or e.y + b.b > ULDl[e.box].dim.b or e.z + b.h >         ULDl[e.box].dim.h or ULDl[e.box].weight + b.weight > ULDl[e.box].maxWt)
        return true;
    
    // check collision of the given packages with all other packages of same ULD
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

void Solver::solve()
{
#ifndef GENETIC
    sort(data.begin(), data.end(), this->sorter.val);
#endif
    for (int i = 0; i < ULDl.size(); i++)
        ep[pair<int, pair<int, pair<int, int>>>(i, pair<int, pair<int, int>>(0, pair<int, int>(0, 0)))] = pair<int, pair<int, int>>(ULDl[i].dim.l, pair<int, int>(ULDl[i].dim.b, ULDl[i].dim.h));
    for (int i = 0; i < data.size(); i++)
    {
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
                if (checkCollision(e, p)) // checks for overlapping if a package is placed on this ULD
                    continue;
                p.isPriority = b.isPriority;
                int score = this->merit.val(e, p, this);
                if (best.first < score)
                    best = pair<int, pair<coords, Box>>(score, pair<coords, Box>(e, p));
            }
        // update vals
        if (best.first == -INF)
            continue;
        if (data[i].isPriority)
            ULDHasPriority[best.second.first.box] = true;
        placement[i] = best.second;
        ULDPackages[best.second.first.box].insert(i);
        gravity_pull(i);
        addEP(i);
        update(i);
    }
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
    surfaces[b].insert(make_pair(placement[i].first.z + placement[i].second.h, make_pair(make_pair(placement[i].first.x, placement[i].first.y), make_pair(placement[i].first.x + placement[i].second.l, placement[i].first.y + placement[i].second.b))));
}

vector<coords> Solver::getCOM()
{
    vector<coords> r(ULDl.size());
    for (int i = 0; i < r.size(); i++)
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

pair<int, pair<int, int>> Solver::getResidueSpace(coords src)
{
    pair<int, pair<int, int>> ret(beamprojectXPos(src).x - src.x, pair<int, int>(beamprojectYPos(src).y - src.y, beamprojectZPos(src).z - src.z));
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

void Solver::addEP2(int i)
{
    // ep.erase(pair<int,pair<int,int>>(placement[i].first.x,pair<int,int>(placement[i].first.y,placement[i].first.z)));
    coords ob1;
    auto p = placement[i];
    // placement[i].first=def;
    ob1.x = p.first.x + placement[i].second.l;
    ob1.y = p.first.y;
    ob1.z = p.first.z + p.second.h;
    vector<pair<pair<int, int>, pair<int, int>>> faces, faces_checked;
    for (int j = 0; j < i; j++)
    {
        faces.push_back(make_pair(make_pair(placement[j].first.x, placement[j].first.y), make_pair(placement[j].first.y + placement[j].second.b, placement[j].first.z + placement[j].second.h)));
    }
    sort(faces.begin(), faces.end(), check);
    for (int j = 0; j < i; j++)
    {
        int x1 = faces[j].first.first;
        int z1 = faces[j].second.second;
        if (ob1.z < z1 || ob1.x > x1)
            continue;
        if (((ob1.y >= faces[j].first.second) && (ob1.y <= faces[j].second.first)) == false)
            continue;
        int flag = 0;
        for (auto yy : placement)
        {
            int x = yy.first.x;
            int y = yy.first.y;
            int z = yy.first.z;
            if (ob1.x < x && x < x1 && z1 > yy.first.z && z1 < yy.first.z + yy.second.h)
                flag = 1;
        }
        if (flag == 0)
        {
            coords d;
            d.x = ob1.x;
            d.y = ob1.y;
            d.z = faces[j].second.second;
            auto r = getResidueSpace(d);
            if (r.first * r.second.first * r.second.second != 0)
                ep[convertCoords(d)] = r;
        }
    }
}
void Solver::addEP(int i)
{
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
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
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
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
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
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
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
    // If the z-coordinate is at the ground level, gravity does not apply
    if (e.z == 0)
        return false;

    // Define the bounding box of the current element
    std::pair<int, int> x1_min = {e.x, e.y};
    std::pair<int, int> x1_max = {e.x + b.l, e.y + b.b};

    int overlappingSurfacesCount = 0;

    // Count the number of surfaces at the same box and z-coordinate
    for (const auto &surface : surfaces[e.box])
    {
        if (surface.first == e.z)
            overlappingSurfacesCount++;
    }

    // If no overlapping surfaces are found at the z-coordinate, gravity applies
    if (overlappingSurfacesCount == 0)
        return true;

    // Check for overlapping surfaces with the current box
    for (const auto &surface : surfaces[e.box])
    {
        if (surface.first != e.z)
            continue;

        auto [z, coords] = surface;
        auto [minCoords, maxCoords] = coords;

        std::pair<int, int> x2_min = minCoords;
        std::pair<int, int> x2_max = maxCoords;

        // Check for overlap between bounding boxes
        if (x1_max.first > x2_min.first && x1_min.first < x2_max.first &&
            x1_max.second > x2_min.second && x1_min.second < x2_max.second)
        {
            return false; // Overlap found, gravity does not apply
        }
    }

    return true; // No overlap, gravity applies
}
void Solver::gravity_pull(int i)
{
    auto x1_min = make_pair(placement[i].first.x, placement[i].first.y);
    auto x1_max = make_pair(placement[i].first.x + placement[i].second.l, placement[i].first.y + placement[i].second.b);

    int m = 0;
    // Find the maximum z-coordinate of overlapping surfaces below the current box

    for (auto x : surfaces[placement[i].first.box])
    {
        auto p = x;
        auto x2_min = make_pair(p.second.first.first, p.second.first.second);
        auto x2_max = make_pair(p.second.second.first, p.second.second.second);

        // Check for overlap and ensure the surface is below the current z-coordinate
        bool is_overlapping = x1_max.first > x2_min.first &&
                              x1_min.first < x2_max.first &&
                              x1_max.second > x2_min.second &&
                              x1_min.second < x2_max.second;

        if (is_overlapping && p.first <= placement[i].first.z)
        {
            m = max(m, p.first);
        }
    }
    // Update the z-coordinate of the placement to the highest valid surface level
    placement[i].first.z = m;
}

int residueFunc(coords c, Box b, Solver *s)
{
    int r = 0;
    r += (1LL * (s->ULDHasPriority[c.box]) * 100000000000LL) * b.isPriority;
    float relativeDifference = (s->ep[convertCoords(c)].first - b.l) / 1.0 / s->ep[convertCoords(c)].first + (s->ep[convertCoords(c)].second.first - b.b) / 1.0 / s->ep[convertCoords(c)].second.first + weightz * (s->ep[convertCoords(c)].second.second - b.h) / 1.0 / s->ep[convertCoords(c)].second.second;
    relativeDifference *= 1000000;
    //    float relativeDifference =(s->ep[convertCoords(c)].first - b.l)+(s->ep[convertCoords(c)].second.first - b.b)+0.1*(s->ep[convertCoords(c)].second.second - b.h);
    r += relativeDifference;
    return r;
}
void ScoredSolver::solve()
{
    // Initial Solve
    insertionCounter.assign(data.size() + 5, 0);
#ifndef GENETIC
    sort(data.begin(), data.end(), this->sorter.val);
#endif
    for (int i = 0; i < ULDl.size(); i++)
        ep[pair<int, pair<int, pair<int, int>>>(i, pair<int, pair<int, int>>(0, pair<int, int>(0, 0)))] = pair<int, pair<int, int>>(ULDl[i].dim.l, pair<int, int>(ULDl[i].dim.b, ULDl[i].dim.h));
    for (int i = 0; i < data.size(); i++)
    {
        // Construct Economy Package and Box Map
        if (!data[i].isPriority)
        {
            boxMap[data[i].ID] = &data[i];
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
                if (checkCollision(e, p))
                    continue;
                p.isPriority = b.isPriority;
                int score = this->merit.val(e, p, this);
                if (best.first < score)
                    best = pair<int, pair<coords, Box>>(score, pair<coords, Box>(e, p));
            }
        // update vals
        if (best.first == -INF)
            continue;
        cout << insertionCounter.size() << " " << data[i].ID << endl;
        cout.flush();
        insertionCounter[data[i].ID] += 1;
        if (data[i].isPriority)
        {
            ULDHasPriority[best.second.first.box] = true;
        }
        else
        {
            lastInsertionSet.insert(data[i].ID);
            lastInsertion.push_back(data[i].ID);
        }
        placement[i] = best.second;
        ULDPackages[best.second.first.box].insert(i);
        gravity_pull(i);
        addEP(i);
        update(i);
    }

    // Initialising the scores
    for (auto i : economyPackages)
    {
        if (lastInsertionSet.find(i) == lastInsertionSet.end())
        {
            score[i] = 3.0 * boxMap[i]->cost;
        }
        else
        {
            score[i] = boxMap[i]->cost;
        }
    }
    for (int i = 1; i <= iterations; i++)
    {
        cout << "Iteration " << i << " started" << endl;
        cout.flush();
        update_scores(i);
        optimize(i);
        cout << "Iteration " << i << " had a cost " << this->cost() << endl;
    }
}

/*
 * i = number of iterations completed
 */
void ScoredSolver::update_scores(int i)
{

    pair<double, int> worst_loaded = {100000000.0, -1}, best_unloaded = {0.0, -1};
    for (auto id : economyPackages)
    {
        if (lastInsertionSet.find(id) == lastInsertionSet.end())
        {
            double theta = (1.0 * boxMap[id]->cost) / (boxMap[id]->l * boxMap[id]->b * boxMap[id]->h * (1 + i - insertionCounter[id]));
            // cout << "Theta: "<< theta << " " << id << endl;
            if (theta > best_unloaded.first)
            {
                best_unloaded = {theta, id};
            }
        }
        else
        {
            double myu = (1.0 * boxMap[id]->cost) / (boxMap[id]->l * boxMap[id]->b * boxMap[id]->h * (1 + insertionCounter[id]));
            if (myu < worst_loaded.first)
            {
                worst_loaded = {myu, id};
            }
        }
    }
    // Found highest myu and theta
    // cout << "Reached here" << endl;
    // cout.flush();
    if (worst_loaded.second != -1 && best_unloaded.second != -1)
    {
        score[worst_loaded.second] *= (1 - alpha);
        score[best_unloaded.second] *= (1 + beta);

        cout << "Score of " << best_unloaded.second << ":" << score[best_unloaded.second] << endl;
        swap(score[worst_loaded.second], score[best_unloaded.second]);
        cout << "Swapped " << worst_loaded.second << " and " << best_unloaded.second << "\n";
        cout << "Score of " << best_unloaded.second << ":" << score[best_unloaded.second] << endl;
    }
    else
    {
        cout << "No possible swaps " << worst_loaded.second << " " << best_unloaded.second << endl;
        return;
    }
}

void ScoredSolver::optimize(int _iter)
{
    // Re-Setup the solver
    placement.clear();
    ULDPackages.clear();
    surfaces.clear();
    ULDHasPriority.clear();
    lastInsertion.clear();
    def.x = def.y = def.z = def.box = -1;
    Box def_;
    def_.l = def_.b = def_.h = -1;
    ULDl = this->originalUldList;
    placement.assign(data.size(), pair<coords, Box>(def, def_));
    ULDPackages.assign(ULDl.size(), set<int>());
    surfaces.assign(ULDl.size(), set<pair<int, pair<pair<int, int>, pair<int, int>>>>());
    ULDHasPriority.assign(ULDl.size(), false);

    sort(data.begin(), data.end(), [&](Box a, Box b)
         {
        if (a.isPriority && !b.isPriority)
            return true;
        if (!a.isPriority && b.isPriority)
            return false;
        if(score[a.ID]!=score[b.ID])return score[a.ID]>score[b.ID];
        if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
        return a.l*a.b*a.h > b.l*b.b*b.h; });
    cout << "Data ordering:" << endl;
    for (auto it : data)
    {
        cout << it.ID << "," << score[it.ID] << " ";
        // cout << it.ID << " ";
    }
    cout << endl;
    for (int i = 0; i < ULDl.size(); i++)
        ep[pair<int, pair<int, pair<int, int>>>(i, pair<int, pair<int, int>>(0, pair<int, int>(0, 0)))] = pair<int, pair<int, int>>(ULDl[i].dim.l, pair<int, int>(ULDl[i].dim.b, ULDl[i].dim.h));
    for (int i = 0; i < data.size(); i++)
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
                if (checkCollision(e, p))
                    continue;
                p.isPriority = b.isPriority;
                int scores = this->merit.val(e, p, this);
                if (best.first < scores)
                    best = pair<int, pair<coords, Box>>(scores, pair<coords, Box>(e, p));
            }
        // update vals
        if (best.first == -INF)
            continue;
        insertionCounter[data[i].ID] += 1;
        if (data[i].isPriority)
        {
            ULDHasPriority[best.second.first.box] = true;
        }
        else
        {
            lastInsertionSet.insert(data[i].ID);
            lastInsertion.push_back(data[i].ID);
        }
        placement[i] = best.second;
        ULDPackages[best.second.first.box].insert(i);
        gravity_pull(i);
        addEP(i);
        update(i);
    }
}