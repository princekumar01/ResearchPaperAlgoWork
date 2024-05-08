#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Structure to represent a taxi
struct Taxi {
    int current_location;
    bool heading_to_destination;
};

// Function to calculate distance between two points
double calculateDistance(int point1, int point2) {
    // Replace this with your actual distance calculation logic
    return abs(point1 - point2);
}
    
// Function to find the nearest hub to the destination
int findNearestHub(const vector<int>& hubs, int destination) {
    int nearest_hub = -1;
    double nearest_hub_distance = INFINITY;
    for (int hub : hubs) {
        double dest_to_hub_distance = calculateDistance(destination, hub);
        if (dest_to_hub_distance < nearest_hub_distance) {
            nearest_hub_distance = dest_to_hub_distance;
            nearest_hub = hub;
        }
    }
    return nearest_hub;
}

// Function to assign a taxi for package delivery
Taxi assignTaxi(const vector<Taxi>& taxis, int current_location, int nearest_hub, bool& taxi_assigned) {
    for (const Taxi& taxi : taxis) {
        if (calculateDistance(taxi.current_location, nearest_hub) < calculateDistance(current_location, nearest_hub)) {
            taxi_assigned = true;
            return taxi;
        }
        if (taxi.heading_to_destination) {
            taxi_assigned = true;
            return taxi;
        }
    }
    taxi_assigned = false;
    return {}; // Return an empty taxi object if no taxi is assigned
}

// Main function implementing the DesCloser algorithm
Taxi DesCloser(int current_location, int destination, const vector<Taxi>& taxis, const vector<int>& hubs, double threshold_distance) {
    // Find the nearest hub to the destination
    int nearest_hub = findNearestHub(hubs, destination);

    // Assign a taxi for package delivery
    bool taxi_assigned;
    Taxi assigned_taxi = assignTaxi(taxis, current_location, nearest_hub, taxi_assigned);

    // If no taxi is available and nearest hub is within threshold distance, wait for a taxi from nearest hub
    if (!taxi_assigned && nearest_hub != -1 && calculateDistance(destination, nearest_hub) < threshold_distance) {
        // Wait until a taxi starts from nearest hub to destination
        // For simplicity, we don't implement the waiting mechanism here
        // Instead, we assume the package is immediately assigned to a taxi from the nearest hub
        assigned_taxi.current_location = nearest_hub;
        assigned_taxi.heading_to_destination = true;
        taxi_assigned = true;
    }

    // If no taxi is assigned, return an empty taxi object
    if (!taxi_assigned) {
        cout << "No taxi available currently." << endl;
        return {};
    }

    return assigned_taxi;
}

int main() {
    // Sample data
    int current_location = 10;
    int destination = 30;
    vector<Taxi> taxis = {{15, false}, {20, true}, {5, false}}; // Sample list of taxis
    vector<int> hubs = {25, 35, 40}; // Sample list of hubs
    double threshold_distance = 15.0; // Sample threshold distance

    // Call the DesCloser algorithm
    Taxi assigned_taxi = DesCloser(current_location, destination, taxis, hubs, threshold_distance);

    // Print the assigned taxi
    if (assigned_taxi.current_location != -1) {
        cout << "Assigned taxi for package delivery:" << endl;
        cout << "Current Location: " << assigned_taxi.current_location << endl;
        cout << "Heading to Destination: " << (assigned_taxi.heading_to_destination ? "Yes" : "No") << endl;
    }

    return 0;
}
