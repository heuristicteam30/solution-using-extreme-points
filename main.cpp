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
    freopen("ULD.in", "r" , stdin);
    For(i,ULDList.size()){
        ULDList[i].weight=0;
        cin>>ULDList[i].dim.l>>ULDList[i].dim.b>>ULDList[i].dim.h>>ULDList[i].maxWt;ULDList[i].maxBound.x =ULDList[i].maxBound.y=ULDList[i].maxBound.z = 0;
        ULDList[i].com.x = ULDList[i].dim.l/2;ULDList[i].com.y = ULDList[i].dim.b/2;ULDList[i].com.z = ULDList[i].dim.h/2;
    }
//    freopen("/Users/agupta/Desktop/q/cpp/cpp/outpus.out", " gw" , stdout);
    freopen("package.in", "r" , stdin);
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
    cout.flush();
    freopen("result.csv", "w" , stdout);
    For(i,dat.size()){
        if(s.placement[i].first.x == -1){
            cout << "P-"<<s.data[i].ID<<",None,-1,-1,-1,-1,-1,-1\n";
        }
        else{
            cout << "P-"<<s.data[i].ID<<",U"<<s.placement[i].first.box+1<<","<<s.placement[i].first.x<<","<<s.placement[i].first.y<<","<<s.placement[i].first.z<<","<<s.placement[i].second.l+s.placement[i].first.x<<","<<s.placement[i].second.b+s.placement[i].first.y<<","<<s.placement[i].second.h+s.placement[i].first.z<<"\n";
        }
        // cout<<s.data[i].ID<<","<<s.placement[i].first.box+1<<","<<s.placement[i].first.x<<","<<s.placement[i].first.y<<","<<s.placement[i].first.z<<","<<s.placement[i].second.l+s.placement[i].first.x<<","<<s.placement[i].second.b+s.placement[i].first.y<<","<<s.placement[i].second.h+s.placement[i].first.z<<"\n";
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
