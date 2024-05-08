#include<bits/stdc++.h>
#include <lp_lib.h>
using namespace std;
//#define d2r (M_PI / 180.0)
const double d2r=0.0174532925199433;
    // Declare ifstream objects
    std::ifstream file1("beijing_adj.txt");
    std::ifstream file2("beijing_sent_matched_timed200.txt");

   
    // Declare ofstream object for output
    std::ofstream outputFile("output.txt");
   
// creating a template function to use pair in unordered set/map
struct hash_pair {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& p) const {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ hash2;
    }
};

void takeInput(vector<double> &double_list){  // to read input from beijing_adj file
    string double_string;
    //file1>>double_string;
    getline(file1, double_string);
    //cout<<double_string<<"\n";
    string s="";
    int commaCnt=0;
    for(int i=0;i<double_string.length();i++){
        if(double_string[i]!=','){
            s+=double_string[i];
        }
        else{
            commaCnt++;
            double_list.push_back(stold(s));
            s="";
        }
        if(commaCnt==4){
           break; 
        }
    }
    if(double_string.length()>0){
        double_list.push_back(stod(double_string));
    } 
}
void tripInput(int &src,int &dest){ // to take input from trip file
    std::string input;
    // Use std::getline to read a line of input until newline
    std::getline(file2, input);
    if(input[0]!='['){
       return; 
    }
    for(int i=1;i<input.length();i++){
        if(input[i]==','){
           break; 
        }
        src=(src*10)+(input[i]-'0');  
    }
    string s="";
    int flag=0;
    for(int i=input.length()-1;i>=0;i--){
        if(input[i]=='['){
           break; 
        }
        if(input[i]==','){
           flag++; 
        } 
        if(flag==3){
           s+=input[i]; 
        }
    }
    reverse(s.begin(),s.end());
    dest=stoi(s);
}
//calculate haversine distance for linear distance
double haversine_km(double lat1, double long1, double lat2, double long2)
{
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * c;

    return d;
}

void preprocessing_Approach2(unordered_map<int,int> &S,unordered_map<int,unordered_set<int>> &OC,unordered_map<int,unordered_set<int>> &EC,unordered_map<pair<int,int>,int,hash_pair> &T,unordered_map<int,unordered_set<pair<int,double>,hash_pair>>& adjIndegree,unordered_map<int,unordered_set<pair<int,double>,hash_pair>> &adjOutdegree,double thresholdDistance){
     for(auto it:T){
        int source=it.first.first;
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
        unordered_map<int,double> distance;
        distance[source] = 0;
        // Push the source node into the priority queue
        pq.push({0, source});
        // Dijkstra's algorithm
        while(!pq.empty()){
            int u = pq.top().second; 
            if(pq.top().first > thresholdDistance){
            break;
            }
            pq.pop();
            OC[source].insert(u);     
            S[u]++;
            for(auto edge:adjIndegree[u]){
                int v = edge.first;
                double w = edge.second;
                if(distance.find(v)==distance.end()){
                   distance[v]=thresholdDistance;
                }
                if(distance[u] + w < distance[v]||distance[u]+w == thresholdDistance) { // checking the current node distance from source node 
                   distance[v] = distance[u] + w;
                   pq.push({distance[v], v});
                }
            }
        }

        int destination=it.first.second;
        while(!pq.empty()){ // to empty the priority queue
            pq.pop();
        }
        distance.clear();
        distance[destination] = 0;
        // Push the source node into the priority queue
        pq.push({0, destination});
        // Dijkstra's algorithm
        while(!pq.empty()){
            int u = pq.top().second; 
            if(pq.top().first > thresholdDistance){
            break;
            }
            pq.pop();
            EC[destination].insert(u);     
            S[u]++; 
            for(auto edge:adjOutdegree[u]) {
                int v = edge.first;
                double w = edge.second;
                if(distance.find(v)==distance.end()){
                   distance[v]=thresholdDistance;
                }
                if(distance[u] + w < distance[v]||distance[u]+w == thresholdDistance) { // checking the current node distance from source node 
                   distance[v] = distance[u] + w;
                   pq.push({distance[v], v});
                }
            }
        }
    }
}


void optimalAlgorithm(unordered_map<pair<int,int>,int,hash_pair> &T,unordered_map<int,unordered_set<int>> &OC,unordered_map<int,unordered_set<int>> &EC,unordered_map<int,int> &S,unordered_map<int,int> &Ho){
    
    map<int,int> oj,ej;
    for(auto it:OC){
        oj[it.first]=0;
        if(S.find(it.first)!=S.end()){
          oj[it.first]=1;
        }
    }
    for(auto it:EC){
        ej[it.first]=0;
        if(S.find(it.first)!=S.end()){
          ej[it.first]=1;
        }
    }
    // Create a new LP problem
    lprec *lp = make_lp(0, 2); // 0 rows, 2 columns

    // Define the objective function: U = U1 + U2 + ...
    double objective[3] = {0}; // The first element is always 0
    set_obj_fn(lp, objective);

    // Define the constraints
    // Constraint counter
    int constraint_counter = 0;

    // Constraint 1: Uj <= oj for all Tj in T
    for (const auto& entry : oj) {
        auto tj = entry.first;
        auto oj_val = entry.second;
        add_constraint(lp, new double[3]{0, 1, 0}, LE, oj_val);
        constraint_counter++;
    }

    // Constraint 2: Uj <= ej for all Tj in T
    for (const auto& entry : ej) {
        auto tj = entry.first;
        auto ej_val = entry.second;
        add_constraint(lp, new double[3]{0, 0, 1}, LE, ej_val);
        constraint_counter++;
    }

    // Constraint 3: oj <= sum(Xi) for Si in OC(Tj) for all Tj in T
    for (const auto& entry : OC) {
        auto tj = entry.first;
        auto& oc = entry.second;
        double* coeffs = new double[oc.size() + 1];
        coeffs[0] = 0;
        for (const auto& si : oc) {
            coeffs[si + 1] = 1; // Coefficients for binary variables Xi
        }
        add_constraint(lp, coeffs, LE, oj[tj]);
        constraint_counter++;
        delete[] coeffs;
    }

    // Constraint 4: ej <= sum(Xi) for Si in EC(Tj) for all Tj in T
    for (const auto& entry : EC) {
        auto tj = entry.first;
        auto& ec = entry.second;
        double* coeffs = new double[ec.size() + 1];
        coeffs[0] = 0;
        for (const auto& si : ec) {
            coeffs[si + 1] = 1; // Coefficients for binary variables Xi
        }
        add_constraint(lp, coeffs, LE, ej[tj]);
        constraint_counter++;
        delete[] coeffs;
    }

    // Constraint 5: Sum of binary variables xi = k + Ho
    // Coefficients for binary variables Xi
    double* coeffs = new double[Ho.size() + 1];
    coeffs[0] = -k;
    for (const auto& si : Ho) {
        coeffs[si + 1] = 1;
    }
    add_constraint(lp, coeffs, EQ, 0);
    constraint_counter++;
    delete[] coeffs;

    // Constraint 6: Each task Si belongs to the set SUHo

    for (const auto& si : S) {
        double* coeffs = new double[Ho.size() + 1];
        coeffs[0] = 0;
        for (const auto& x : Ho) {
            coeffs[x + 1] = (x == si) ? 1 : 0;
        }
        add_constraint(lp, coeffs, EQ, 1);
        constraint_counter++;
        delete[] coeffs;
    }

    // Constraint 7: Xi = 1 for all Si in Ho
    for (const auto& si : Ho) {
        double* coeffs = new double[Ho.size() + 1];
        coeffs[0] = 0;
        for (const auto& x : Ho) {
            coeffs[x + 1] = (x == si) ? 1 : 0;
        }
        add_constraint(lp, coeffs, EQ, 1);
        constraint_counter++;
        delete[] coeffs;
    }

    // Set binary variables
    set_binary(lp, TRUE);

    // Set optimization direction
    set_maxim(lp);

    // Solve the problem
    int result = solve(lp);
    if (result == 0) {
        // Get the optimal solution
        double *vars = new double[3];
        get_variables(lp, vars);
        std::cout << "Optimal solution found:\n";
        std::cout << "U1 = " << vars[1] << ", U2 = " << vars[2] << "\n";
        delete[] vars;
    } else {
        std::cerr << "No optimal solution found\n";
    }

    // Free memory and delete LP
    delete_lp(lp);

}

int main(){
    // to speed up the input and output process 
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);
    cout << fixed << setprecision(7);
    outputFile << fixed << setprecision(7);

    // Check if the files are opened successfully
    if (!file1.is_open() || !file2.is_open() || !outputFile.is_open()) {
        std::cerr << "Error opening files" << std::endl;
        return 1; // Return an error code
    }
    
    int nodes=0,edges=0;
    vector<double> input;
    takeInput(input);
    nodes=input[0];
    edges=input[1];
    
    map<int,pair<double,double>> nodesLatLong;// map to store Latitude and Longitude of each node
    for(int i=0;i<nodes;i++){
        input.clear();
        takeInput(input); // taking input all nodes from beijing_adj file
        nodesLatLong[input[0]]={input[1],input[2]};
    }

    unordered_map<int,unordered_set<pair<int,double>,hash_pair>> adjIndegree,adjOutdegree;
    for(int i=0;i<edges;i++){
        input.clear();
        takeInput(input); // taking input all edges from beijing_adj file
        int source=input[1];
        int destination=input[2];
        double distance=haversine_km(nodesLatLong[source].first,nodesLatLong[source].second,nodesLatLong[destination].first,nodesLatLong[destination].second);
        if(input[3]==0){
           adjOutdegree[source].insert({destination,distance});
           adjOutdegree[destination].insert({source,distance});
           adjIndegree[source].insert({destination,distance});
           adjIndegree[destination].insert({source,distance}); 
        }  
        else{
            adjOutdegree[source].insert({destination,distance});
            adjIndegree[destination].insert({source,distance});
        }
    }

    unordered_map<pair<int,int>,int,hash_pair> T;// to storing end point of trip
    for(int i=0;i<320;i++){
        int src=0,dest=0;
        tripInput(src,dest); // taking input all trip end point from mapping file
        if(i%2==1){
          T[{src,dest}]++; 
        }
    }     
    double thresholdDistance=0.1;
    unordered_map<int,unordered_set<int>> OC,EC;
    unordered_map<int,int> S,Ho;
    //S -> Candiadte Hub location, Ho -> Intial Hub
    preprocessing_Approach2(S,OC,EC,T,adjIndegree,adjOutdegree,thresholdDistance);
    double time1=(float)clock() / CLOCKS_PER_SEC ;
    cerr << "time taken in input process : " << time1<< " secs" << endl;    
    int k=10;
    cout<<optimalAlgorithm(T,OC,EC,S,Ho); 	
    double time2=(float)clock() / CLOCKS_PER_SEC ;
    cerr << "\ntime taken to run the Algorithm : " << time2-time1 << " secs" << endl;
    return 0;
}
