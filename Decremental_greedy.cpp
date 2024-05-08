#include<bits/stdc++.h>
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

void updateUtilitiesDecrementalGreedy(int s0,unordered_map<int,int> &H,unordered_map<int,int> &oj,unordered_map<int,int> &ej,unordered_map<int,unordered_set<int>> &OC,unordered_map<int,unordered_set<int>> &EC,unordered_map<pair<int,int>,int,hash_pair> &T,unordered_map<int,int> &MU,unordered_map<int,int> &delta){
    for(auto it:T){
        int source=it.first.first,destination=it.first.second,trip=it.second;
        if((oj[source]==1 && ej[destination]==1) && (OC[source].find(s0)!=OC[source].end())){
           OC[source].erase(s0);
           if(OC[source].size()==0){
              oj[source]=0;
              for(auto it1:EC[destination]){
                delta[it1]-=1;
             }
           }
           else if(OC[source].size()==1){
                   auto it1=OC[source].begin();
                   MU[*it1]+=1;
           } 
        }

        if((oj[source]==1 && ej[destination]==1) && (EC[destination].find(s0)!=EC[destination].end())){
           EC[destination].erase(s0);
           if(EC[destination].size()==0){
              ej[destination]=0;
              for(auto it1:OC[source]){
                delta[it1]-=1;
             }
           }
           else if(EC[destination].size()==1){
                auto it1=EC[destination].begin();
                MU[*it1]+=1;
           } 
        }
    }
}

void intialiseDecrementalGreedy(unordered_map<int,int> &oj,unordered_map<int,int> &ej,unordered_map<int,int> &MU,unordered_map<int,int> &delta,unordered_map<int,int> &S,unordered_map<int,unordered_set<int>> &OC,unordered_map<int,unordered_set<int>> &EC,unordered_map<int,int> &H,unordered_map<int,int> &Ho,unordered_map<pair<int,int>,int,hash_pair> &T){
    //  for(auto it:Ho){
    //     S[it.first]=1;
    //  } 
     for(auto it:S){
        MU[it.first]=0;
        delta[it.first]=0;
     }
     for(auto it:T){
        int source=it.first.first,destination=it.first.second,trip=it.second;
        oj[source]=1;
        ej[destination]=1;
        for(auto si:OC[source]){
            delta[si]+=1;
            if(OC[source].size()==1){
               MU[si]+=1; 
            }
        }
        for(auto si:EC[destination]){
            delta[si]+=1;
            if(EC[destination].size()==1){
               MU[si]+=1; 
            }
        } 
     }
}

int decrementalGreedy(int k,unordered_map<int,int> &Ho,unordered_map<int,int> &S,unordered_map<int,unordered_set<int>> &OC,unordered_map<int,unordered_set<int>> &EC,unordered_map<pair<int,int>,int,hash_pair> &T,double thresholdDistance){
     unordered_map<int,int> H,delta,MU,oj,ej;
     // H -> Hub, U -> Utility, PU -> Potential Utility
     for(auto it:S){
        if(Ho.find(it.first)==Ho.end())
        H[it.first]=1;
     }
     int n=H.size();
     //cout<<"size of S "<<n<<"\n";
     intialiseDecrementalGreedy(oj,ej,MU,delta,S,OC,EC,H,Ho,T);
     
     for(int i=0;i<n-k;i++){
        //  outputFile2<<"----------------------------------------------------------MU\n";
        //  for(auto it:MU){
        //     outputFile2<<it.first<<" "<<it.second<<"\n";
        //  }

        //  outputFile2<<"---------------------------------------------------------------------delta\n";
        //  for(auto it:delta){
        //        outputFile2<<it.first<<" "<<it.second<<"\n";
        //  }

        int count=0,node=INT_MAX,minMarginalUtility=INT_MAX;
        for(auto it:MU){
            if(H.find(it.first)!=H.end()){
               if(it.second<minMarginalUtility){
                  minMarginalUtility=it.second;
                  node=it.first;
                  count=1; 
                }
                else if(it.second==minMarginalUtility){
                    count++;
                    node=it.first;
                }
            }
        }
        
        unordered_map<int,int> minUtilityHub;
        if(count==0){
          minUtilityHub=MU;
        }
        else if(count==1){
            //cout<<node<<"<- ";
            H.erase(node);
            updateUtilitiesDecrementalGreedy(node,H,oj,ej,OC,EC,T,MU,delta);
            continue;
        }
        else if(count>1){
               for(auto it:MU){
                  if(it.second==minMarginalUtility and H.find(it.first)!=H.end()){
                     minUtilityHub[it.first]++;
                  }
               }
        }
        // to find the highest potential Utility in the candidate hub location where max utility ties up
        node=INT_MIN;
        int minTotalDegree=INT_MAX;
        for(auto it:delta){
            int totalDegree=it.second;
            if(minUtilityHub.find(it.first)!=minUtilityHub.end()){
              if(totalDegree<minTotalDegree){
                minTotalDegree=totalDegree;
                node=it.first; 
               }
               else if(minTotalDegree==totalDegree){
                  node=max(node,it.first);
               }
            }
        }
        //cout<<node<<"<- ";
        H.erase(node);
        updateUtilitiesDecrementalGreedy(node,H,oj,ej,OC,EC,T,MU,delta);
    }

    // for(auto it:H){
    //     if(Ho.find(it.first)==Ho.end()){
    //       cout<<it.first<<" ";
    //     }
    // }
    for(auto it:Ho){
        H[it.first]++;
    }
    int totalTripCover=0;
    for(auto it:T){
        int source=it.first.first;
        int destination=it.first.second;
        int count=0;
        for(auto it1:OC[source]){
            if(H.find(it1)!=H.end()){
               count++;
               break; 
            }
        }
        for(auto it1:EC[destination]){
            if(H.find(it1)!=H.end()){
               count++;
               break; 
            }
        }
        if(count==2){
            totalTripCover+=it.second; 
        }
    }
    return totalTripCover;
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
    cout<<decrementalGreedy(k,Ho,S,OC,EC,T,thresholdDistance); 	
    double time2=(float)clock() / CLOCKS_PER_SEC ;
    cerr << "\ntime taken to run the Algorithm : " << time2-time1 << " secs" << endl;
    return 0;
}
