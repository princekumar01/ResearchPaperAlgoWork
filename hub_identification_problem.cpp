#include<bits/stdc++.h>
using namespace std;
const double d2r=0.0174532925199433;

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

void tripInput(int &src,int &dest,map<int,pair<double,double>> &nodesLatLon){ // to take input from trip file
    std::string input;
    // Use std::getline to read a line of input until newline
    std::getline(file2, input);
    if(input[0]!='['){
       return; 
    }
    int cnt=0;
    string s1="",s2="";
    for(int i=1;i<input.length();i++){
        if(input[i]==','){
           cnt++;
           if(cnt==3){
            break;
           }
           continue; 
        }
        if(cnt==0)
           src=(src*10)+(input[i]-'0');
        else if(cnt==1){
             s1+=input[i];
        }
        else{
            s2+=input[i];
        }     
    }
    nodesLatLon[src]={stold(s1),stold(s2)};
    string s="";
    s1="";
    s2="";
    int flag=0;
    for(int i=input.length()-1;i>=0;i--){
        if(input[i]=='['){
           break; 
        }
        if(input[i]==','){
           flag++; 
           continue;
        } 
        if(flag==3){
           s+=input[i]; 
        }
        else if(flag==2){
             s1+=input[i];
        }
        else if(flag==1){
             s2+=input[i];
        }
    }
    reverse(s.begin(),s.end());
    reverse(s1.begin(),s1.end());
    reverse(s2.begin(),s2.end());
    dest=stoi(s);
    nodesLatLon[dest]={stold(s1),stold(s2)};
}

double haversine_km(double lat1, double long1, double lat2, double long2){
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * c;

    return d;
}

// Function to identify hubs
vector<int> identifyHubs(vector<pair<double,int>> ranked, double Th,map<int,pair<double,double>> &nodesLatLon) {
    vector<int> hubs;
    for(int i=0;i<ranked.size();i++){
       int flag=1;
       int v=ranked[i].second;
       for(int j=0;j<hubs.size();j++){
           if(haversine_km(nodesLatLon[v].first,nodesLatLon[v].second,nodesLatLon[hubs[j]].first,nodesLatLon[hubs[j]].second)<Th){
             flag=0;
             break;
           }
       }
       if(flag==1){
         hubs.push_back(v);
       }
    }
    return hubs;
}


// Function to calculate graph entropy
double calculateGraphEntropy(unordered_map<pair<int,int>,int,hash_pair> &T,int vi) {
    double entropy = 0.0,numerator_pvi=0,denominator_pvi=0;
    for(auto it:T){
        if(it.first.first!=vi && it.first.second!=vi)
           denominator_pvi+=it.second;
    }
    for(auto it1:T){
        vi=it1.first.first;
        for(auto it:T){
           if(it.first.first==vi)
              numerator_pvi+=it.second;
        }
         double pvi=numerator_pvi/denominator_pvi;
         entropy+=(pvi*(log(pvi)));    
    }
    return entropy;
}

// Function to calculate node entropy
double calculateNodeEntropy(unordered_map<pair<int,int>,int,hash_pair> &T,int vi) {
    double entropy = 0.0,numerator_pvi=0,denominator_pvi=0;
    for(auto it:T){
        if(it.first.first==vi)
          denominator_pvi+=it.second;
    }
    for(auto it:T){
        if(it.first.first==vi){
           numerator_pvi=it.second;
           double pvi=numerator_pvi/denominator_pvi;
           entropy+=(pvi*(log(pvi))); 
        }  
    }
    return entropy;
}

// Function to rank nodes based on total pickups and drop-offs
vector<pair<double,int>> rankNodes(unordered_map<pair<int,int>,int,hash_pair> &T, int k) {
    vector<pair<double,int>> rankedNodes;
    map<int,int> freq;
    for(auto it:T){
        freq[it.first.first]+=it.second;
        freq[it.first.second]+=it.second;
    }
    vector<pair<int,int>> freqCopy;
    for(auto it:freq){
        freqCopy.push_back({it.second,it.first});
    }
    sort(freqCopy.rbegin(),freqCopy.rend());
    for(int i=0;i<k;i++){
       double nodeEntropy=calculateNodeEntropy(T,freqCopy[i].first);
       double graphEntropy=calculateGraphEntropy(T,freqCopy[i].first);
       rankedNodes.push_back({(nodeEntropy/graphEntropy),freqCopy[i].second});
    }    
    sort(rankedNodes.rbegin(),rankedNodes.rend());
    return rankedNodes;
}


int main() {
    // Sample data
    unordered_map<pair<int,int>,int,hash_pair> T;// to storing end point of trip
    unordered_map<int,int> S;
    map<int,pair<double,double>> nodesLatLong;// map to store Latitude and Longitude of each node
    for(int i=0;i<320;i++){
        int src=0,dest=0;
        tripInput(src,dest,nodesLatLong); // taking input all trip end point from mapping file
        if(i%2==1){
          T[{src,dest}]++; 
          S[src]++;
          S[dest]++;
        }
    }
    // Rank nodes based on total pickups and drop-offs
    int k = 5; // Number of top nodes to select
    vector<pair<double,int>> influentialFactor = rankNodes(T, k);
    vector<int> hubs=identifyHubs(influentialFactor,k,nodesLatLong);  
    // Calculate Influential Factor for each selected node
    


    
    return 0;
}
